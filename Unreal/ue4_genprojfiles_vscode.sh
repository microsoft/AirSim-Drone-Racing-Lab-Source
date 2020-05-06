# Generate VS Code UE project files (overwrites .vscode\settings.json)
echo Generating VS Code project files with environment variable UE4_22_ROOT=$UE4_22_ROOT
$UE4_ROOT/Engine/Build/BatchFiles/Linux/Build.sh -projectfiles -vscode -project="$PWD/AirSimExe.uproject" -game -engine