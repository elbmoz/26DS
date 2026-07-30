$ErrorActionPreference = "Stop"
$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$agent = Join-Path $scriptDirectory "vision_agent.py"
$receiver = Join-Path $scriptDirectory "stream_receiver.py"

python $agent restart --deploy
if ($LASTEXITCODE -ne 0) {
    throw "MaixCAM deploy/restart failed with code $LASTEXITCODE"
}

python $receiver
if ($LASTEXITCODE -ne 0) {
    throw "stream receiver exited with code $LASTEXITCODE"
}
