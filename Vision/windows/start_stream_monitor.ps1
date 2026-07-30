$ErrorActionPreference = "Stop"
$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$receiver = Join-Path $scriptDirectory "stream_receiver.py"

python $receiver
if ($LASTEXITCODE -ne 0) {
    throw "stream receiver exited with code $LASTEXITCODE"
}
