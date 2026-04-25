#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通用 YAW 格式解码器

解码由 HikCamera 或 MindVision 相机节点生成的 .yaw 原始数据流文件。

文件格式:
  - ASCII 头部: YAWFMT, WIDTH, HEIGHT, PIXELTYPE/MEDIATYPE, FPS, INTERVAL, ===DATA===
  - 二进制数据帧: <8字节时间戳(us, LE)><4字节长度(LE)><原始数据>
  - ASCII 尾部: ===META===, FRAMES <数量>

用法:
  python3 decode_yaw.py input.yaw [--out_dir decoded] [--mode ffmpeg|images|raw]
  
模式:
  - ffmpeg: 使用 ffmpeg 生成视频文件 (默认)
  - images: 保存每一帧为图像文件
  - raw: 直接导出原始帧数据
"""

import argparse
import os
import struct
import sys


def parse_header(f):
    """解析 YAW 文件头部"""
    width = height = pixel_type = 0
    fps = 0.0
    interval = 1
    
    while True:
        line = f.readline()
        if not line:
            raise RuntimeError("Unexpected EOF while reading header")
        if isinstance(line, bytes):
            line = line.decode('ascii', errors='ignore')
        line = line.strip()
        
        if line == '===DATA===':
            break
            
        if line.startswith('WIDTH '):
            try:
                width = int(line.split()[1])
            except Exception:
                pass
        elif line.startswith('HEIGHT '):
            try:
                height = int(line.split()[1])
            except Exception:
                pass
        elif line.startswith('PIXELTYPE ') or line.startswith('MEDIATYPE '):
            try:
                pixel_type = int(line.split()[1])
            except Exception:
                pass
        elif line.startswith('FPS '):
            try:
                fps = float(line.split()[1])
            except Exception:
                pass
        elif line.startswith('INTERVAL '):
            try:
                interval = int(line.split()[1])
            except Exception:
                pass
                
    return width, height, pixel_type, fps, interval


def decode_frame_heuristic(data, width, height, force_color=False, bayer_pattern='RG'):
    """启发式解码帧数据 (无 SDK 时的回退方案)

    如果 `force_color` 为 True，则尝试把单通道数据按 Bayer 解码为 RGB。
    bayer_pattern 可选值: 'RG','GB','GR','BG'，默认 'RG'.
    """
    import numpy as np
    import cv2

    data_len = len(data)

    # RGB24
    if data_len == 3 * width * height:
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
        return arr

    # 单通道数据（可能是灰度或Bayer）
    if data_len == width * height:
        arr1 = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
        if force_color:
            # 选择 Bayer 转换代码
            code = cv2.COLOR_BayerRG2RGB
            if bayer_pattern == 'GB':
                code = cv2.COLOR_BayerGB2RGB
            elif bayer_pattern == 'GR':
                code = cv2.COLOR_BayerGR2RGB
            elif bayer_pattern == 'BG':
                code = cv2.COLOR_BayerBG2RGB

            try:
                rgb = cv2.cvtColor(arr1, code)
                return rgb
            except Exception:
                # 若 debayer 失败，回退为灰度扩展
                return np.dstack([arr1, arr1, arr1]).astype('uint8')
        else:
            # 仅扩展灰度到三通道
            return np.dstack([arr1, arr1, arr1]).astype('uint8')

    return None


def main():
    parser = argparse.ArgumentParser(description='Decode YAW format raw camera files')
    parser.add_argument('infile', help='Input .yaw file')
    parser.add_argument('--out_dir', '-o', default='decoded', help='Output directory')
    parser.add_argument('--mode', choices=['ffmpeg', 'images', 'raw'], default='ffmpeg',
                       help='Decoding mode: ffmpeg (video), images (per-frame), raw (dump)')
    parser.add_argument('--out_video', default='output.mp4', help='Output video filename (ffmpeg mode)')
    parser.add_argument('--save_frames', action='store_true', 
                       help='Also save frames as images (ffmpeg mode)')
    parser.add_argument('--fps', type=float, default=0.0, 
                       help='Override FPS (0 = use header value or default 30)')
    parser.add_argument('--format', choices=['png', 'jpg'], default='png',
                       help='Image format for saved frames')
    parser.add_argument('--color', action='store_true',
                       help='Force decode single-channel frames to color (debayer)')
    parser.add_argument('--bayer-pattern', choices=['RG','GB','GR','BG'], default='RG',
                       help='Bayer pattern to use when debayering (default RG)')
    parser.add_argument('--rotate', type=int, choices=[0,1,2,3], default=0,
                       help='Rotate output images by i*90 degrees clockwise (i in [0,3])')
    
    args = parser.parse_args()
    
    infile = args.infile
    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)
    
    mode = args.mode
    out_video = os.path.join(out_dir, args.out_video)
    save_frames = args.save_frames
    fps_override = args.fps
    img_format = args.format
    
    print(f"Decoding {infile}...")
    
    try:
        import numpy as np
        import cv2
    except ImportError:
        print("ERROR: OpenCV (cv2) and numpy are required for decoding")
        print("Install with: pip install opencv-python numpy")
        sys.exit(1)
    
    frame_idx = 0
    ffmpeg_proc = None
    cv_writer = None
    
    try:
        with open(infile, 'rb') as f:
            width, height, pixel_type, fps, interval = parse_header(f)
            
            if fps_override > 0.0:
                fps = fps_override
            if fps == 0.0:
                fps = 30.0
                
            print(f"Header: {width}x{height}, PixelType={pixel_type}, FPS={fps}, Interval={interval}")
            
            # 设置输出
            if mode == 'ffmpeg':
                if width <= 0 or height <= 0:
                    raise RuntimeError('Invalid width/height in header')
                
                import shutil
                import subprocess
                
                ffmpeg_path = shutil.which('ffmpeg')
                # If rotation is 90 or 270 degrees, swap input size for ffmpeg
                in_w, in_h = (width, height) if (args.rotate % 2) == 0 else (height, width)
                if ffmpeg_path:
                    cmd = [
                        ffmpeg_path, '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                        '-s', f'{in_w}x{in_h}', '-r', str(fps), '-i', '-',
                        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', out_video
                    ]
                    print(f"Launching ffmpeg: {' '.join(cmd)}")
                    ffmpeg_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
                else:
                    # 回退到 OpenCV VideoWriter
                    print("ffmpeg not found, using OpenCV VideoWriter")
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    vw_size = (int(width), int(height)) if (args.rotate % 2) == 0 else (int(height), int(width))
                    cv_writer = cv2.VideoWriter(out_video, fourcc, float(fps), vw_size)
                    if not cv_writer.isOpened():
                        raise RuntimeError('Failed to open OpenCV VideoWriter')
            
            # 读取帧
            while True:
                hdr = f.read(8 + 4)  # timestamp + length
                if not hdr or len(hdr) < 12:
                    break
                
                ts, length = struct.unpack('<QI', hdr)
                data = f.read(length)
                if not data or len(data) < length:
                    print(f"Warning: Incomplete frame {frame_idx}")
                    break
                
                # 解码帧（可选择强制解码为彩色）
                rgb_frame = decode_frame_heuristic(data, width, height,
                                                   force_color=args.color,
                                                   bayer_pattern=args.bayer_pattern)
                
                if rgb_frame is None:
                    if mode == 'raw':
                        raw_path = os.path.join(out_dir, f'frame_{frame_idx:06d}.raw')
                        with open(raw_path, 'wb') as rf:
                            rf.write(data)
                        print(f"Saved raw frame {frame_idx} -> {raw_path}")
                    else:
                        print(f"Warning: Could not decode frame {frame_idx}")
                    frame_idx += 1
                    continue
                
                # 输出帧
                # Apply rotation (clockwise k*90) if requested
                rotate_k = args.rotate % 4
                if rotate_k != 0:
                    # np.rot90 rotates counter-clockwise; use negative to rotate clockwise
                    import numpy as _np
                    rgb_frame = _np.rot90(rgb_frame, -rotate_k)

                if mode == 'ffmpeg':
                    # RGB 格式用于 ffmpeg
                    rgb_bytes = rgb_frame.tobytes()

                    if ffmpeg_proc:
                        try:
                            ffmpeg_proc.stdin.write(rgb_bytes)
                        except Exception as e:
                            print(f"Error writing to ffmpeg: {e}")
                    elif cv_writer:
                        # OpenCV 需要 BGR
                        bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                        cv_writer.write(bgr_frame)

                    if save_frames:
                        img_path = os.path.join(out_dir, f'frame_{frame_idx:06d}.{img_format}')
                        bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                        cv2.imwrite(img_path, bgr_frame)

                elif mode == 'images':
                    img_path = os.path.join(out_dir, f'frame_{frame_idx:06d}.{img_format}')
                    bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(img_path, bgr_frame)
                    print(f"Saved frame {frame_idx} -> {img_path}")
                
                frame_idx += 1
                if frame_idx % 100 == 0:
                    print(f"Processed {frame_idx} frames...")
        
        # 清理
        if ffmpeg_proc:
            ffmpeg_proc.stdin.close()
            ffmpeg_proc.wait()
            print(f"Video saved to {out_video}")
        if cv_writer:
            cv_writer.release()
            print(f"Video saved to {out_video}")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    print(f"Done. Processed {frame_idx} frames.")


if __name__ == '__main__':
    main()
