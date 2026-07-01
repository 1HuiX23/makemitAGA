/*
 * =================================================================================================
 * FormulaRenderService.cs
 * =================================================================================================
 *
 * 【职责】
 *   使用 CSharpMath 排版 LaTeX，再由 SkiaSharp 绘制透明位图、裁剪透明边缘并编码为 PNG。
 *   本文件只处理托管内存和 Skia 对象，不创建 Unity Texture2D / GameObject。
 *
 * 【数据流】
 *   LaTeX -> MathPainter -> 4096x1024 透明工作画布 -> Alpha 边界扫描 -> 裁剪 -> PNG byte[]。
 *
 * 【为什么先画大画布再裁剪】
 *   CSharpMath 的复杂分式、积分、根号上下边界难以仅靠字符串长度预测。
 *   大画布可避免公式被截断，Alpha 裁剪则消除多余透明空白，令 1～4 字宽判断更准确。
 *
 * 【维护陷阱】
 *   1. SKBitmap 使用 BGRA8888，Alpha 是每像素第 4 个字节；换颜色格式必须同步改扫描逻辑。
 *   2. WorkingWidth/Height 越大，单次临时内存越高；不要无上限扩大。
 *   3. 本方法捕获异常并通过 error 返回，调用方会回退成普通文字，不能让异常逃出协程。
 *   4. PNG byte[] 之后必须在 Unity 主线程转为 Texture2D。
 * =================================================================================================
 */
using System;
using System.Runtime.InteropServices;
using CSharpMath.SkiaSharp;
using SkiaSharp;

namespace MakemitAGA.Dialogue
{
    internal sealed class FormulaRasterResult
    {
        public byte[] PngBytes;
        public int PixelWidth;
        public int PixelHeight;
    }

    /// <summary>
    /// 使用 CSharpMath + SkiaSharp 在透明 BGRA 位图中绘制公式，
    /// 然后扫描 Alpha 边界并裁剪，避免输出大面积透明空白。
    /// </summary>
    internal static class FormulaRenderService
    {
        private const int WorkingWidth = 4096;
        private const int WorkingHeight = 1024;
        private const int DrawX = 96;
        private const int DrawY = WorkingHeight / 2;
        private const int CropPadding = 10;
        private const float RenderFontSize = 96f;

        /// <summary>
        /// 尝试渲染一条不含外层 $ 定界符的 LaTeX。成功时返回裁剪后的 PNG 和像素尺寸。
        /// </summary>
        public static bool TryRender(
            string latex,
            out FormulaRasterResult result,
            out string error)
        {
            result = null;
            error = null;

            if (string.IsNullOrWhiteSpace(latex))
            {
                error = "LaTeX is empty.";
                return false;
            }

            try
            {
                var info = new SKImageInfo(
                    WorkingWidth,
                    WorkingHeight,
                    SKColorType.Bgra8888,
                    SKAlphaType.Premul);

                using var bitmap = new SKBitmap(info);
                using var canvas = new SKCanvas(bitmap);

                canvas.Clear(SKColors.Transparent);

                var painter = new MathPainter
                {
                    LaTeX = latex,
                    FontSize = RenderFontSize,
                    TextColor = SKColors.White,
                    AntiAlias = true
                };

                painter.Draw(canvas, DrawX, DrawY);
                canvas.Flush();

                if (!TryFindAlphaBounds(
                    bitmap,
                    out SKRectI alphaBounds))
                {
                    error =
                        "CSharpMath drew no non-transparent pixels.";
                    return false;
                }

                SKRectI crop = ExpandAndClamp(
                    alphaBounds,
                    CropPadding,
                    bitmap.Width,
                    bitmap.Height);

                int width = crop.Width;
                int height = crop.Height;

                if (width <= 0 || height <= 0)
                {
                    error = "Formula crop has invalid dimensions.";
                    return false;
                }

                using var cropped = new SKBitmap(
                    width,
                    height,
                    SKColorType.Bgra8888,
                    SKAlphaType.Premul);

                using (var cropCanvas = new SKCanvas(cropped))
                {
                    cropCanvas.Clear(SKColors.Transparent);
                    cropCanvas.DrawBitmap(
                        bitmap,
                        crop,
                        new SKRect(0f, 0f, width, height));
                    cropCanvas.Flush();
                }

                using SKImage image = SKImage.FromBitmap(cropped);
                using SKData data = image.Encode(
                    SKEncodedImageFormat.Png,
                    100);

                if (data == null || data.Size <= 0)
                {
                    error = "Skia PNG encoding returned no data.";
                    return false;
                }

                result = new FormulaRasterResult
                {
                    PngBytes = data.ToArray(),
                    PixelWidth = width,
                    PixelHeight = height
                };

                return true;
            }
            catch (Exception e)
            {
                error = e.GetType().Name + ": " + e.Message;
                return false;
            }
        }

        /// <summary>扫描非透明像素范围。这里依赖 SKColorType.Bgra8888 的字节布局。</summary>
        private static bool TryFindAlphaBounds(
            SKBitmap bitmap,
            out SKRectI bounds)
        {
            bounds = SKRectI.Empty;

            int byteCount = bitmap.ByteCount;
            if (byteCount <= 0)
                return false;

            byte[] pixels = new byte[byteCount];
            Marshal.Copy(
                bitmap.GetPixels(),
                pixels,
                0,
                pixels.Length);

            int minX = bitmap.Width;
            int minY = bitmap.Height;
            int maxX = -1;
            int maxY = -1;
            int rowBytes = bitmap.RowBytes;

            for (int y = 0; y < bitmap.Height; y++)
            {
                int row = y * rowBytes;

                for (int x = 0; x < bitmap.Width; x++)
                {
                    // BGRA8888：Alpha 位于第 4 字节。
                    int alphaIndex = row + x * 4 + 3;

                    if (alphaIndex < 0 ||
                        alphaIndex >= pixels.Length ||
                        pixels[alphaIndex] == 0)
                    {
                        continue;
                    }

                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
            }

            if (maxX < minX || maxY < minY)
                return false;

            bounds = new SKRectI(
                minX,
                minY,
                maxX + 1,
                maxY + 1);

            return true;
        }

        private static SKRectI ExpandAndClamp(
            SKRectI rect,
            int padding,
            int width,
            int height)
        {
            return new SKRectI(
                Math.Max(0, rect.Left - padding),
                Math.Max(0, rect.Top - padding),
                Math.Min(width, rect.Right + padding),
                Math.Min(height, rect.Bottom + padding));
        }
    }
}