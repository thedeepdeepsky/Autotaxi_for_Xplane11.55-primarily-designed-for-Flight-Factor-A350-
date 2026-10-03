param(
    [string]$CsvPath = (Join-Path ([System.IO.Path]::GetTempPath()) 'autotaxi-gate93-map.csv'),
    [string]$OutputDirectory = (Join-Path $PSScriptRoot '../build-release/arrival-map')
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$rows = @(Import-Csv -LiteralPath $CsvPath | ForEach-Object {
    [pscustomobject]@{ Kind = $_.kind; Id = $_.id;
        X = [double]::Parse($_.x, $culture); Y = [double]::Parse($_.y, $culture);
        Style = if ($_.style) { [int]$_.style } else { 0 };
        Lead = if ($_.lead) { [int]$_.lead } else { 0 } }
})
$route = @($rows | Where-Object Kind -eq 'route')
$groups = @($rows | Group-Object { $_.Kind + ':' + $_.Id })
[System.IO.Directory]::CreateDirectory($OutputDirectory) | Out-Null
function Render-Map([string]$name, [double]$cx, [double]$cy, [double]$span) {
    $bitmap = [System.Drawing.Bitmap]::new(1200, 900)
    $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
    $graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $graphics.Clear([System.Drawing.Color]::FromArgb(30, 32, 33))
    $scale = 1100 / $span
    $centerPen = [System.Drawing.Pen]::new([System.Drawing.Color]::FromArgb(248, 184, 30), 1.5)
    $centerLeadInPen = [System.Drawing.Pen]::new([System.Drawing.Color]::FromArgb(255, 194, 24), 2.0)
    $boundaryPen = [System.Drawing.Pen]::new([System.Drawing.Color]::FromArgb(52, 56, 61), 1.15)
    $whiteMarkingPen = [System.Drawing.Pen]::new([System.Drawing.Color]::FromArgb(190, 197, 204), 1.2)
    $markerPen = [System.Drawing.Pen]::new([System.Drawing.Color]::FromArgb(245, 248, 250), 2.8)
    $routePen = [System.Drawing.Pen]::new([System.Drawing.Color]::Turquoise, 2)
    try {
        foreach ($group in $groups) {
            $points = [System.Collections.Generic.List[System.Drawing.PointF]]::new()
            foreach ($point in $group.Group) {
                $points.Add([System.Drawing.PointF]::new(
                    [single](600 + ($point.X - $cx) * $scale),
                    [single](450 - ($point.Y - $cy) * $scale)))
            }
            if ($points.Count -gt 1) {
                $first = $group.Group[0]
                if ($first.Kind -eq 'route') {
                    $pen = $routePen
                } elseif ($first.Kind -eq 'marker') {
                    $pen = $markerPen
                } elseif ([int]$first.Style -in @(1, 7, 51, 57, 101, 105)) {
                    $pen = $centerPen
                    if ([int]$first.Lead -eq 1) { $pen = $centerLeadInPen }
                } elseif ([int]$first.Style -in @(19, 20, 22, 30, 31)) {
                    $pen = $whiteMarkingPen
                } else {
                    $pen = $boundaryPen
                }
                $graphics.DrawLines($pen, $points.ToArray())
            }
        }
        $bitmap.Save((Join-Path $OutputDirectory ($name + '.png')), [System.Drawing.Imaging.ImageFormat]::Png)
    } finally {
        $graphics.Dispose()
        $bitmap.Dispose()
        $centerPen.Dispose()
        $centerLeadInPen.Dispose()
        $boundaryPen.Dispose()
        $whiteMarkingPen.Dispose()
        $markerPen.Dispose()
        $routePen.Dispose()
    }
}
$xmin = ($route | Measure-Object X -Minimum).Minimum
$xmax = ($route | Measure-Object X -Maximum).Maximum
$ymin = ($route | Measure-Object Y -Minimum).Minimum
$ymax = ($route | Measure-Object Y -Maximum).Maximum
Render-Map 'overview' (($xmin + $xmax) / 2) (($ymin + $ymax) / 2) ([Math]::Max($xmax - $xmin, ($ymax - $ymin) * 1.4) + 200)
$walked = 0.0
$next = 250.0
for ($i = $route.Count - 2; $i -ge 0 -and $next -le 2250; $i--) {
    $dx = $route[$i + 1].X - $route[$i].X
    $dy = $route[$i + 1].Y - $route[$i].Y
    $walked += [Math]::Sqrt($dx * $dx + $dy * $dy)
    if ($walked -ge $next) {
        Render-Map ('final-' + [int]$next) $route[$i].X $route[$i].Y 650
        $next += 500
    }
}
Write-Output ('Local audit images: ' + [System.IO.Path]::GetFullPath($OutputDirectory))
