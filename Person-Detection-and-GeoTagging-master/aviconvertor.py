import ffmpeg

def compress_video(input_path, output_path, crf=28, preset="slow", scale=None):
    """
    Compresses a video using ffmpeg-python.

    Args:
        input_path (str): Path to input video.
        output_path (str): Path to save compressed output.
        crf (int): Constant Rate Factor (lower = better quality, higher = smaller size).
        preset (str): Speed/efficiency tradeoff ('ultrafast'→'veryslow').
        scale (tuple): Optional (width, height) to resize video.
    """
    try:
        stream = ffmpeg.input(input_path)
        if scale:
            stream = ffmpeg.filter(stream, 'scale', scale[0], scale[1])
        stream = ffmpeg.output(
            stream, output_path,
            vcodec='libx264',
            crf=crf,
            preset=preset,
            acodec='aac',
            movflags='+faststart'
        )
        ffmpeg.run(stream, overwrite_output=True)
        print(f"✅ Compressed '{input_path}' → '{output_path}' (CRF={crf}, preset={preset})")
    except ffmpeg.Error as e:
        print(f"❌ Compression failed: {e}")

# Example:
compress_video(
    "/home/suryansh/All-Coding-FIles/YOLO-Trained/runs/detect/predict2/test.avi",
    "compressed_video.mp4",
    crf=28,          # increase to 30–32 for even smaller size
    preset="slow",   # or 'veryslow' for max compression
    scale=(1280, 720)  # optional: resize to 720p
)
