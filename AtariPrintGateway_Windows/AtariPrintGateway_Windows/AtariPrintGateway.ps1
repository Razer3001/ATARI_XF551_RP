param(
  [string]$Prefix = "http://+:8080/",
  [string]$PrinterName = "Brother DCP-T720DW"
)

Add-Type -AssemblyName System.Drawing

function Get-PrinterNameToUse {
  param([string]$Requested)

  $installed = [System.Drawing.Printing.PrinterSettings]::InstalledPrinters

  if (![string]::IsNullOrWhiteSpace($Requested)) {
    foreach ($p in $installed) {
      if ($p -eq $Requested -or $p -like "*$Requested*") {
        return $p
      }
    }
  }

  $settings = New-Object System.Drawing.Printing.PrinterSettings
  return $settings.PrinterName
}

$SelectedPrinter = Get-PrinterNameToUse -Requested $PrinterName

Write-Host "ATARI Print Gateway v31" -ForegroundColor Cyan
Write-Host "Escuchando en: $Prefix"
Write-Host "Impresora:     $SelectedPrinter"
Write-Host "Endpoints:"
Write-Host "  GET  /status   (alias /health)"
Write-Host "  GET  /test"
Write-Host "  POST /print    (alias /api/print/text)"
Write-Host "  POST /api/print/job  (JSON rawBase64 simple)"
Write-Host ""
Write-Host "Cambio v31: /api/print/job renderiza ATASCII crudo con glifos 8x8."
Write-Host "Preserva caracteres especiales Atari e inverso; /print queda como texto legacy."
Write-Host ""
Write-Host "Para detener: CTRL+C"
Write-Host ""

function Send-TextResponse {
  param($Context, [int]$StatusCode, [string]$Text, [string]$ContentType = "text/plain; charset=utf-8")

  $bytes = [System.Text.Encoding]::UTF8.GetBytes($Text)
  $Context.Response.StatusCode = $StatusCode
  $Context.Response.ContentType = $ContentType
  $Context.Response.ContentLength64 = $bytes.Length
  $Context.Response.KeepAlive = $false
  $Context.Response.Headers["Connection"] = "close"
  $Context.Response.OutputStream.Write($bytes, 0, $bytes.Length)
  $Context.Response.OutputStream.Close()
}


function Convert-RawAtasciiToLines {
  param([byte[]]$Bytes, [int]$Columns = 40)

  if ($Columns -lt 1) { $Columns = 40 }
  $lines = New-Object System.Collections.ArrayList
  $line = New-Object 'System.Collections.Generic.List[byte]'

  foreach ($b in $Bytes) {
    if ($b -eq 0x9B -or $b -eq 0x0D -or $b -eq 0x0A) {
      [void]$lines.Add($line.ToArray())
      $line.Clear()
      continue
    }

    $line.Add([byte]$b)
    if ($line.Count -ge $Columns) {
      [void]$lines.Add($line.ToArray())
      $line.Clear()
    }
  }

  if ($line.Count -gt 0 -or $lines.Count -eq 0) {
    [void]$lines.Add($line.ToArray())
  }

  return @($lines)
}

function Get-AtasciiGlyphRows {
  param($Job)

  if ($Job -and $Job.glyphRowsHex -and $Job.glyphRowsHex.Count -ge 128) {
    return $Job.glyphRowsHex
  }

  # Fallback mínimo si llega un Gateway job antiguo sin tabla.
  # El ESP32 F25 siempre envía glyphRowsHex, por lo que normalmente no se usa.
  $blank = @("00","00","00","00","00","00","00","00")
  $rows = @()
  for ($i = 0; $i -lt 128; $i++) { $rows += ,$blank }
  return $rows
}

function Draw-AtasciiGlyph {
  param(
    [System.Drawing.Graphics]$Graphics,
    [float]$X,
    [float]$Y,
    [byte]$Code,
    $GlyphRows,
    [float]$PixelSize,
    [System.Drawing.Brush]$Brush
  )

  $base = [int]($Code -band 0x7F)
  $inverse = (($Code -band 0x80) -ne 0)

  for ($gy = 0; $gy -lt 8; $gy++) {
    $rowHex = "00"
    try { $rowHex = [string]$GlyphRows[$base][$gy] } catch { $rowHex = "00" }
    if ([string]::IsNullOrWhiteSpace($rowHex)) { $rowHex = "00" }
    $row = 0
    try { $row = [Convert]::ToInt32($rowHex, 16) } catch { $row = 0 }

    for ($gx = 0; $gx -lt 8; $gx++) {
      $on = (($row -band (0x80 -shr $gx)) -ne 0)
      if ($inverse) { $on = -not $on }
      if ($on) {
        $Graphics.FillRectangle(
          $Brush,
          [float]($X + ($gx * $PixelSize)),
          [float]($Y + ($gy * $PixelSize)),
          [float]($PixelSize + 0.05),
          [float]($PixelSize + 0.05)
        )
      }
    }
  }
}

function Print-AtariRawJob {
  param($Job)

  $bytes = [System.Convert]::FromBase64String([string]$Job.rawBase64)
  $columns = 40
  if ($Job.columns) { $columns = [int]$Job.columns }
  if ($columns -lt 1) { $columns = 40 }

  $script:rawAtariLines = Convert-RawAtasciiToLines -Bytes $bytes -Columns $columns
  $script:rawAtariLineIndex = 0
  $script:rawAtariGlyphRows = Get-AtasciiGlyphRows -Job $Job
  $script:rawAtariBrush = [System.Drawing.Brushes]::Black

  # Escala en centésimas de pulgada. Ajusta para que 40/80/132 columnas quepan razonablemente.
  if ($columns -le 40) { $script:rawAtariPixel = [float]2.0 }
  elseif ($columns -le 80) { $script:rawAtariPixel = [float]1.0 }
  else { $script:rawAtariPixel = [float]0.65 }

  $script:rawAtariCellW = [float](8.0 * $script:rawAtariPixel)
  $script:rawAtariCellH = [float]((8.0 * $script:rawAtariPixel) + 2.0)

  $doc = New-Object System.Drawing.Printing.PrintDocument
  $doc.DocumentName = "Atari P: ATASCII Raw Job"
  $doc.PrinterSettings.PrinterName = $SelectedPrinter

  $doc.add_PrintPage({
    param($sender, $e)

    $e.Graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::None
    $e.Graphics.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::NearestNeighbor
    $e.Graphics.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::Half

    $left = [float]$e.MarginBounds.Left
    $top = [float]$e.MarginBounds.Top
    $right = [float]$e.MarginBounds.Right
    $bottom = [float]$e.MarginBounds.Bottom
    $y = $top

    while ($script:rawAtariLineIndex -lt $script:rawAtariLines.Count) {
      if (($y + $script:rawAtariCellH) -gt $bottom) {
        $e.HasMorePages = $true
        return
      }

      $line = [byte[]]$script:rawAtariLines[$script:rawAtariLineIndex]
      $x = $left
      foreach ($b in $line) {
        if (($x + $script:rawAtariCellW) -gt $right) { break }
        Draw-AtasciiGlyph -Graphics $e.Graphics -X $x -Y $y -Code ([byte]$b) -GlyphRows $script:rawAtariGlyphRows -PixelSize $script:rawAtariPixel -Brush $script:rawAtariBrush
        $x += $script:rawAtariCellW
      }

      $y += $script:rawAtariCellH
      $script:rawAtariLineIndex++
    }

    $e.HasMorePages = $false
  })

  $doc.Print()
  $doc.Dispose()
}

function Print-AtariText {
  param([string]$Text)

  if ([string]::IsNullOrWhiteSpace($Text)) {
    $Text = " "
  }

  $script:atariLines = (($Text -replace "`r`n", "`n") -replace "`r", "`n").Split("`n")
  $script:atariLineIndex = 0
  $script:atariFont = New-Object System.Drawing.Font("Consolas", 10)
  $script:atariBrush = [System.Drawing.Brushes]::Black

  $doc = New-Object System.Drawing.Printing.PrintDocument
  $doc.DocumentName = "Atari P: Print Job"
  $doc.PrinterSettings.PrinterName = $SelectedPrinter

  $doc.add_PrintPage({
    param($sender, $e)

    $left = [float]$e.MarginBounds.Left
    $top = [float]$e.MarginBounds.Top
    $bottom = [float]$e.MarginBounds.Bottom
    $y = $top
    $lineHeight = [float]($script:atariFont.GetHeight($e.Graphics) + 2)

    while ($script:atariLineIndex -lt $script:atariLines.Count) {
      if (($y + $lineHeight) -gt $bottom) {
        $e.HasMorePages = $true
        return
      }

      $line = [string]$script:atariLines[$script:atariLineIndex]
      $e.Graphics.DrawString($line, $script:atariFont, $script:atariBrush, $left, $y)
      $y += $lineHeight
      $script:atariLineIndex++
    }

    $e.HasMorePages = $false
  })

  $doc.Print()
  $doc.Dispose()
}

$listener = New-Object System.Net.HttpListener
$listener.Prefixes.Add($Prefix)

try {
  $listener.Start()
} catch {
  Write-Host "No se pudo iniciar HttpListener." -ForegroundColor Red
  Write-Host "Ejecuta PowerShell como Administrador o crea URL ACL:"
  Write-Host "netsh http add urlacl url=$Prefix user=Everyone"
  throw
}

while ($listener.IsListening) {
  $ctx = $listener.GetContext()
  $path = $ctx.Request.Url.AbsolutePath.ToLowerInvariant()
  $method = $ctx.Request.HttpMethod.ToUpperInvariant()

  try {
    if ($method -eq "GET" -and ($path -eq "/status" -or $path -eq "/health")) {
      $json = "{`"ok`":true,`"version`":`"v31`",`"printer`":`"$SelectedPrinter`"}"
      Send-TextResponse -Context $ctx -StatusCode 200 -Text $json -ContentType "application/json; charset=utf-8"
      continue
    }

    if ($method -eq "GET" -and $path -eq "/test") {
      Send-TextResponse -Context $ctx -StatusCode 200 -Text "OK TEST QUEUED"
      Write-Host "[$(Get-Date -Format HH:mm:ss)] Test encolado"
      Print-AtariText -Text "ATARI PRINT GATEWAY TEST`r`nLINEA 2 GATEWAY OK`r`nLINEA 3 BROTHER OK`r`n"
      Write-Host "[$(Get-Date -Format HH:mm:ss)] Test enviado al driver"
      continue
    }

    if ($method -eq "POST" -and ($path -eq "/print" -or $path -eq "/api/print/text")) {
      $reader = New-Object System.IO.StreamReader($ctx.Request.InputStream, [System.Text.Encoding]::UTF8)
      $body = $reader.ReadToEnd()
      $reader.Close()

      $lineCount = ((($body -replace "`r`n", "`n") -replace "`r", "`n").Split("`n")).Count
      Write-Host "[$(Get-Date -Format HH:mm:ss)] Trabajo recibido: $($body.Length) caracteres, $lineCount líneas"

      # Responder al ESP32 antes de llamar al driver.
      # Esto evita que el ESP32 marque READ_TIMEOUT mientras Windows prepara/imprime.
      Send-TextResponse -Context $ctx -StatusCode 200 -Text "OK QUEUED chars=$($body.Length) lines=$lineCount"

      try {
        Print-AtariText -Text $body
        Write-Host "[$(Get-Date -Format HH:mm:ss)] Trabajo enviado al driver: $lineCount líneas" -ForegroundColor Green
      } catch {
        Write-Host "ERROR imprimiendo luego de responder HTTP: $($_.Exception.Message)" -ForegroundColor Red
      }

      continue
    }


    if ($method -eq "POST" -and $path -eq "/api/print/job") {
      $reader = New-Object System.IO.StreamReader($ctx.Request.InputStream, [System.Text.Encoding]::UTF8)
      $body = $reader.ReadToEnd()
      $reader.Close()

      try {
        $job = $body | ConvertFrom-Json
        if ($job.rawBase64) {
          $bytes = [System.Convert]::FromBase64String([string]$job.rawBase64)
          $columns = 40
          if ($job.columns) { $columns = [int]$job.columns }
          $lineCount = (Convert-RawAtasciiToLines -Bytes $bytes -Columns $columns).Count
          Write-Host "[$(Get-Date -Format HH:mm:ss)] ATASCII RAW recibido: $($bytes.Length) bytes, $lineCount líneas, cols=$columns"

          # Responder al ESP32 antes del driver para evitar timeout.
          Send-TextResponse -Context $ctx -StatusCode 200 -Text "OK QUEUED atasciiRaw bytes=$($bytes.Length) lines=$lineCount"

          try {
            Print-AtariRawJob -Job $job
            Write-Host "[$(Get-Date -Format HH:mm:ss)] ATASCII RAW enviado al driver: $lineCount líneas" -ForegroundColor Green
          } catch {
            Write-Host "ERROR imprimiendo ATASCII RAW luego de responder HTTP: $($_.Exception.Message)" -ForegroundColor Red
          }

          continue
        }
      } catch {
        Write-Host "JSON /api/print/job inválido, se imprimirá como texto legacy: $($_.Exception.Message)" -ForegroundColor Yellow
      }

      # Compatibilidad legacy: si no trae rawBase64, imprimimos texto plano.
      $lineCount = ((($body -replace "`r`n", "`n") -replace "`r", "`n").Split("`n")).Count
      Write-Host "[$(Get-Date -Format HH:mm:ss)] Trabajo JSON/text legacy recibido: $($body.Length) caracteres, $lineCount líneas"
      Send-TextResponse -Context $ctx -StatusCode 200 -Text "OK QUEUED legacyText chars=$($body.Length) lines=$lineCount"

      try {
        Print-AtariText -Text $body
        Write-Host "[$(Get-Date -Format HH:mm:ss)] Trabajo legacy enviado al driver: $lineCount líneas" -ForegroundColor Green
      } catch {
        Write-Host "ERROR imprimiendo legacy luego de responder HTTP: $($_.Exception.Message)" -ForegroundColor Red
      }

      continue
    }

    Send-TextResponse -Context $ctx -StatusCode 404 -Text "Not found"
  } catch {
    $msg = $_.Exception.Message
    Write-Host "ERROR: $msg" -ForegroundColor Red
    try {
      Send-TextResponse -Context $ctx -StatusCode 500 -Text "ERROR: $msg"
    } catch {
      # Si la respuesta ya fue enviada, solo registramos.
    }
  }
}
