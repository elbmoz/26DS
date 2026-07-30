$ErrorActionPreference = "Stop"
$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$console = Join-Path $scriptDirectory "operator_console.py"

python $console
if ($LASTEXITCODE -ne 0) {
    throw "operator console exited with code $LASTEXITCODE"
}
