from typing import List, Dict
import numpy as np
from armor_fusion.types import ArmorMeasurement

# Defer heavy sklearn imports until runtime to avoid slowing node startup.
_have_sklearn = None
DBSCAN = None


def cluster_measurements(measurements: List[ArmorMeasurement], eps: float, min_samples: int) -> Dict[int, List[ArmorMeasurement]]:
    """Cluster measurements into groups.

    Tries to use sklearn.DBSCAN. If sklearn is not available, falls back to a simple
    greedy radius-based clustering.
    """
    if len(measurements) == 0:
        return {}

    positions = np.array([m.position for m in measurements])

    # Try to import sklearn lazily the first time clustering is needed.
    global _have_sklearn, DBSCAN
    if _have_sklearn is None:
        try:
            from sklearn.cluster import DBSCAN as _DB
            DBSCAN = _DB
            _have_sklearn = True
        except Exception:
            DBSCAN = None
            _have_sklearn = False

    if _have_sklearn and DBSCAN is not None:
        clustering = DBSCAN(eps=eps, min_samples=min_samples)
        labels = clustering.fit_predict(positions)
        clusters = {}
        for idx, label in enumerate(labels):
            if label == -1:
                continue
            clusters.setdefault(label, []).append(measurements[idx])
        return clusters

    # Fallback simple clustering: greedy cluster by distance to cluster centers
    clusters = {}
    centers = []
    for idx, pos in enumerate(positions):
        placed = False
        for cid, center in enumerate(centers):
            if np.linalg.norm(pos - center) <= eps:
                clusters[cid].append(measurements[idx])
                # update center
                centers[cid] = np.mean([m.position for m in clusters[cid]], axis=0)
                placed = True
                break
        if not placed:
            cid = len(centers)
            centers.append(pos.copy())
            clusters[cid] = [measurements[idx]]

    # Remove clusters with fewer than min_samples
    filtered = {cid: members for cid, members in clusters.items() if len(members) >= max(1, int(min_samples))}
    return filtered


def merge_close_clusters(clusters: Dict[int, List[ArmorMeasurement]], max_cluster_noise: float) -> Dict[int, List[ArmorMeasurement]]:
    if len(clusters) <= 1:
        return clusters
    cluster_centers = {}
    for cid, measurements in clusters.items():
        positions = np.array([m.position for m in measurements])
        cluster_centers[cid] = np.mean(positions, axis=0)
    cluster_ids = list(clusters.keys())
    merged = set()
    merge_map = {}
    for i in range(len(cluster_ids)):
        if cluster_ids[i] in merged:
            continue
        merge_group = [cluster_ids[i]]
        for j in range(i + 1, len(cluster_ids)):
            if cluster_ids[j] in merged:
                continue
            dist = np.linalg.norm(cluster_centers[cluster_ids[i]] - cluster_centers[cluster_ids[j]])
            if dist < max_cluster_noise:
                merge_group.append(cluster_ids[j])
                merged.add(cluster_ids[j])
        for cid in merge_group:
            merge_map[cid] = cluster_ids[i]
    merged_clusters = {}
    for old_id, measurements in clusters.items():
        new_id = merge_map.get(old_id, old_id)
        merged_clusters.setdefault(new_id, []).extend(measurements)
    return merged_clusters
