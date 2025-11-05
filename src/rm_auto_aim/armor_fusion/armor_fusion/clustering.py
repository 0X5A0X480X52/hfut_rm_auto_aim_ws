from typing import List, Dict
import numpy as np
from sklearn.cluster import DBSCAN
from .types import ArmorMeasurement


def cluster_measurements(measurements: List[ArmorMeasurement], eps: float, min_samples: int) -> Dict[int, List[ArmorMeasurement]]:
    if len(measurements) == 0:
        return {}
    positions = np.array([m.position for m in measurements])
    clustering = DBSCAN(eps=eps, min_samples=min_samples)
    labels = clustering.fit_predict(positions)
    clusters = {}
    for idx, label in enumerate(labels):
        if label == -1:
            continue
        clusters.setdefault(label, []).append(measurements[idx])
    return clusters


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
