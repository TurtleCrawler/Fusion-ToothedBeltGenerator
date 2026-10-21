#!/bin/bash
echo "Installing Toothed Belt Generator plugin bundle for Autodesk Fusion..."

BUNDLE_DIR="$HOME/Library/Application Support/Autodesk/ApplicationPlugins/ToothedBeltGenerator.bundle"
CONTENTS_DIR="$BUNDLE_DIR/Contents"

rm -rf "$BUNDLE_DIR"
mkdir -p "$CONTENTS_DIR"

cp -R ToothedBeltGenerator/. "$CONTENTS_DIR/"

cat << 'EOF' > "$BUNDLE_DIR/PackageContents.xml"
<?xml version="1.0" encoding="utf-8"?>
<ApplicationPackage SchemaVersion="1.0" ProductType="Application" AutodeskProduct="Fusion" Name="Toothed Belt Generator" Description="Generate and edit parametric GT1.5, GT2, HTD 5M, and HTD 8M timing belts." AppVersion="1.2.0" Author="Mike Cloutier" AppNameSpace="ToothedBeltGenerator" ProductCode="{8A9F4D2B-5C3E-4A1D-9B7F-2E6D1C8A0B3F}" UpgradeCode="{9B0E5F3C-6D4F-4B2E-8C9A-3F7E2D9B1C4A}">
  <Components>
    <RuntimeRequirements OS="win64|mac" Platform="Fusion" />
    <ComponentEntry AppName="ToothedBeltGenerator" Version="1.2.0" ModuleName="./Contents/ToothedBeltGenerator.manifest" />
  </Components>
</ApplicationPackage>
EOF

echo "Installation complete! Please restart Autodesk Fusion."