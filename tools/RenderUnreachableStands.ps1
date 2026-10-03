[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$TsvPath,
    [Parameter(Mandatory = $true)][string]$OutputDirectory
)

$ErrorActionPreference = 'Stop'
$culture = [Globalization.CultureInfo]::InvariantCulture
$rows = @(Import-Csv -Delimiter "`t" -LiteralPath $TsvPath)
$ramps = @($rows | Where-Object { $_.kind -eq 'ramp' } | ForEach-Object {
    [pscustomobject]@{
        Id = [int]$_.id; Name = $_.name
        X = [double]::Parse($_.x, $culture); Y = [double]::Parse($_.y, $culture)
        Heading = [double]::Parse($_.heading, $culture)
        Reachable = $_.reachable -eq '1'; Reason = $_.reason
    }
})
$unreachable = @($ramps | Where-Object { -not $_.Reachable })
$lineMap = @{}
foreach ($row in ($rows | Where-Object { $_.kind -eq 'line' })) {
    $key = [string]$row.id
    if (-not $lineMap.ContainsKey($key)) {
        $lineMap[$key] = [pscustomobject]@{
            Style = [int]$row.style; Lead = $row.lead -eq '1'; Points = [System.Collections.Generic.List[object]]::new()
        }
    }
    $lineMap[$key].Points.Add([pscustomobject]@{
        X = [double]::Parse($row.x, $culture); Y = [double]::Parse($row.y, $culture)
    })
}
$lines = @($lineMap.Values)
[IO.Directory]::CreateDirectory($OutputDirectory) | Out-Null

function Esc([string]$value) { return [System.Security.SecurityElement]::Escape($value) }
function Number([double]$value) { return $value.ToString('0.0', $culture) }
function MapPoint([double]$x, [double]$y, [double]$cx, [double]$cy, [double]$span, [int]$width, [int]$height) {
    $scale = ($width - 120) / $span
    return [pscustomobject]@{ X = 60 + ($x - $cx) * $scale; Y = $height / 2 - ($y - $cy) * $scale }
}
function LineStyle([int]$style, [bool]$lead) {
    if ($lead) { return @{ Color = '#ffd119'; Width = 3.0 } }
    if ($style -in @(1, 7, 51, 57, 101, 105)) { return @{ Color = '#f8b81e'; Width = 1.4 } }
    if ($style -in @(19, 20, 22, 30, 31)) { return @{ Color = '#bfc5cc'; Width = 1.1 } }
    return @{ Color = '#34383d'; Width = 1.0 }
}
function Bounds() {
    $all = @()
    foreach ($line in $lines) { $all += $line.Points }
    foreach ($ramp in $ramps) { $all += [pscustomobject]@{ X = $ramp.X; Y = $ramp.Y } }
    $xmin = ($all | Measure-Object X -Minimum).Minimum; $xmax = ($all | Measure-Object X -Maximum).Maximum
    $ymin = ($all | Measure-Object Y -Minimum).Minimum; $ymax = ($all | Measure-Object Y -Maximum).Maximum
    return [pscustomobject]@{ X = ($xmin + $xmax) / 2; Y = ($ymin + $ymax) / 2; Span = [Math]::Max($xmax - $xmin, ($ymax - $ymin) * 1.78) + 250 }
}
function WriteMap([string]$path, [double]$cx, [double]$cy, [double]$span, [bool]$overview, $target) {
    $width = 1800; $height = 1050
    $svg = [Text.StringBuilder]::new()
    [void]$svg.AppendLine("<svg xmlns='http://www.w3.org/2000/svg' width='$width' height='$height' viewBox='0 0 $width $height'>")
    [void]$svg.AppendLine("<rect width='$width' height='$height' fill='#1e2021'/>")
    $title = if ($overview) { "ZSSS 36L unreachable stands ($($unreachable.Count))" } else { "ZSSS 36L → $($target.Name) · local final approach" }
    [void]$svg.AppendLine("<text x='32' y='38' fill='#e8edef' font-family='Consolas,monospace' font-size='22'>$(Esc $title)</text>")
    foreach ($line in $lines) {
        if (-not $overview) {
            $near = @($line.Points | Where-Object { [Math]::Abs($_.X - $cx) -lt $span / 2 + 100 -and [Math]::Abs($_.Y - $cy) -lt $span / 2 + 100 })
            if ($near.Count -eq 0) { continue }
        }
        $style = LineStyle $line.Style $line.Lead
        $points = ($line.Points | ForEach-Object { $p = MapPoint $_.X $_.Y $cx $cy $span $width $height; "$(Number $p.X),$(Number $p.Y)" }) -join ' '
        if ($points) { [void]$svg.AppendLine("<polyline points='$points' fill='none' stroke='$($style.Color)' stroke-width='$($style.Width)' stroke-linecap='round' stroke-linejoin='round'/>") }
    }
    foreach ($ramp in $ramps) {
        if (-not $overview -and ([Math]::Abs($ramp.X - $cx) -ge $span / 2 + 80 -or [Math]::Abs($ramp.Y - $cy) -ge $span / 2 + 80)) { continue }
        $p = MapPoint $ramp.X $ramp.Y $cx $cy $span $width $height
        $color = if ($ramp.Reachable) { '#68747b' } else { '#ff4d5e' }
        $radius = if ($target -and $ramp.Id -eq $target.Id) { 10 } elseif ($ramp.Reachable) { 3 } else { 6 }
        [void]$svg.AppendLine("<circle cx='$(Number $p.X)' cy='$(Number $p.Y)' r='$radius' fill='$color' stroke='#111315' stroke-width='1'/>")
        if (($overview -and -not $ramp.Reachable) -or ($target -and $ramp.Id -eq $target.Id)) {
            [void]$svg.AppendLine("<text x='$(Number ($p.X + 9))' y='$(Number ($p.Y - 8))' fill='$color' font-family='Consolas,monospace' font-size='13'>$(Esc $ramp.Name)</text>")
        }
    }
    $origin = MapPoint 0 0 $cx $cy $span $width $height
    [void]$svg.AppendLine("<circle cx='$(Number $origin.X)' cy='$(Number $origin.Y)' r='8' fill='#44d8b5' stroke='#e8edef' stroke-width='2'/>")
    [void]$svg.AppendLine("<text x='$(Number ($origin.X + 12))' y='$(Number ($origin.Y - 10))' fill='#44d8b5' font-family='Consolas,monospace' font-size='14'>RWY 36L start</text>")
    if (-not $overview -and $target) {
        $reason = if ($target.Reason) { $target.Reason } else { 'No route from runway end under current route limits' }
        [void]$svg.AppendLine("<text x='32' y='$(($height - 32))' fill='#ff8b96' font-family='Consolas,monospace' font-size='15'>$(Esc $reason)</text>")
    } else {
        [void]$svg.AppendLine("<text x='32' y='$(($height - 32))' fill='#a9b2b8' font-family='Consolas,monospace' font-size='15'>Red = unreachable under current FF A350 width / pavement / Lead-in rules · grey = reachable</text>")
    }
    [void]$svg.AppendLine('</svg>')
    [IO.File]::WriteAllText($path, $svg.ToString(), [Text.UTF8Encoding]::new($false))
}

$bounds = Bounds
$overviewPath = Join-Path $OutputDirectory 'zsss-36l-unreachable-overview.svg'
WriteMap $overviewPath $bounds.X $bounds.Y $bounds.Span $true $null
$i = 1
foreach ($ramp in $unreachable) {
    $slug = ('{0:D3}-' -f $i) + (($ramp.Name -replace '[^A-Za-z0-9]+', '-') -replace '^-|-$', '')
    $dir = Join-Path $OutputDirectory $slug
    [IO.Directory]::CreateDirectory($dir) | Out-Null
    WriteMap (Join-Path $dir 'local.svg') $ramp.X $ramp.Y 700 $false $ramp
    $i++
}
$manifest = @('# ZSSS 36L unreachable stands', '', "Unreachable count: $($unreachable.Count)", '', '[Airport overview](./zsss-36l-unreachable-overview.svg)', '')
$i = 1
foreach ($ramp in $unreachable) {
    $slug = ('{0:D3}-' -f $i) + (($ramp.Name -replace '[^A-Za-z0-9]+', '-') -replace '^-|-$', '')
    $manifest += "- [$($ramp.Name)](./$slug/local.svg) — $($ramp.Reason)"
    $i++
}
[IO.File]::WriteAllLines((Join-Path $OutputDirectory 'README.md'), $manifest, [Text.UTF8Encoding]::new($false))
Write-Output "Overview: $overviewPath"
Write-Output "Local images: $($unreachable.Count)"
