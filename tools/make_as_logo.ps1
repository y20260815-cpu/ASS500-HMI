Add-Type -AssemblyName System.Drawing

$source = [System.Drawing.Bitmap]::FromFile((Resolve-Path "main/ui/AS_LOGO.png"))
$width = 240
$height = 120
$resized = New-Object System.Drawing.Bitmap($width, $height, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
$graphics = [System.Drawing.Graphics]::FromImage($resized)
$graphics.CompositingMode = [System.Drawing.Drawing2D.CompositingMode]::SourceCopy
$graphics.CompositingQuality = [System.Drawing.Drawing2D.CompositingQuality]::HighQuality
$graphics.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
$graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::HighQuality
$graphics.DrawImage($source, 0, 0, $width, $height)

$bytes = New-Object System.Collections.Generic.List[byte]
for ($y = 0; $y -lt $height; $y++) {
    for ($x = 0; $x -lt $width; $x++) {
        $pixel = $resized.GetPixel($x, $y)
        # LVGL recolors this A8 mask white. Preserve only the PNG alpha so
        # the gray source artwork becomes solid white instead of translucent.
        $bytes.Add([byte]$pixel.A)
    }
}

$lines = New-Object System.Collections.Generic.List[string]
for ($i = 0; $i -lt $bytes.Count; $i += 16) {
    $last = [Math]::Min($i + 15, $bytes.Count - 1)
    $chunk = $bytes[$i..$last] | ForEach-Object { "0x{0:X2}" -f $_ }
    $lines.Add("    " + ($chunk -join ", ") + ",")
}

$cpp = @"
#include "as_logo.h"

alignas(4) static const uint8_t as_logo_data[] = {
$($lines -join "`r`n")
};

const lv_image_dsc_t as_logo = {
    .header = {
        .magic = LV_IMAGE_HEADER_MAGIC,
        .cf = LV_COLOR_FORMAT_A8,
        .flags = 0,
        .w = $width,
        .h = $height,
        .stride = $width,
        .reserved_2 = 0,
    },
    .data_size = sizeof(as_logo_data),
    .data = as_logo_data,
    .reserved = nullptr,
};
"@

[System.IO.File]::WriteAllText((Join-Path (Resolve-Path ".") "main/ui/as_logo.cpp"), $cpp, [System.Text.UTF8Encoding]::new($false))
$graphics.Dispose()
$resized.Dispose()
$source.Dispose()
