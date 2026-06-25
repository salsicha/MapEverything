#!/usr/bin/env python3
"""Check local App Store release readiness for MapEverything.

This script verifies repository-local release inputs. It intentionally reports
App Store Connect account tasks as warnings because privacy labels, screenshots,
agreements, export-compliance answers, and TestFlight groups cannot be checked
from the working tree.
"""

from __future__ import annotations

import argparse
import json
import plistlib
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MapEverything" / "MapEverything.xcodeproj"
PBXPROJ = PROJECT / "project.pbxproj"
INFO_PLIST = ROOT / "MapEverything" / "MapEverything" / "Info.plist"
ENTITLEMENTS = ROOT / "MapEverything" / "MapEverything" / "MapEverything.entitlements"
ASSETS = ROOT / "MapEverything" / "MapEverything" / "Assets.xcassets"
EXPORT_OPTIONS = ROOT / "tools" / "app-store-export-options.plist"

REQUIRED_USAGE_STRINGS = {
    "NSCameraUsageDescription": "camera and AR capture",
    "NSLocationWhenInUseUsageDescription": "GPS, geotiles, and indoor localization",
    "NSBluetoothAlwaysUsageDescription": "BLE beacon telemetry",
    "NSLocalNetworkUsageDescription": "ROS bridge publishing",
}

REQUIRED_DOCS = [
    "docs/app-store-publishing-plan.md",
    "docs/validation-plan.md",
    "docs/geospatial-provider-decision.md",
    "docs/ios-radio-restrictions.md",
    "docs/ros2-companion-package.md",
]

REQUIRED_TOOLS = [
    "tools/app-store-release-check.py",
    "tools/app-store-export-options.plist",
    "tools/run-rosbridge-recorder.py",
    "tools/rosbridge-throughput-benchmark.py",
    "tools/mapeverything-local-bag-to-ros2.py",
]


@dataclass
class Check:
    status: str
    category: str
    name: str
    detail: str


def read_plist(path: Path) -> dict:
    with path.open("rb") as handle:
        return plistlib.load(handle)


def pbx_values(setting: str) -> list[str]:
    if not PBXPROJ.exists():
        return []

    pattern = re.compile(rf"\b{re.escape(setting)}\s*=\s*([^;]+);")
    values = []
    for match in pattern.finditer(PBXPROJ.read_text(encoding="utf-8")):
        values.append(match.group(1).strip().strip('"'))
    return sorted(set(values))


def add(checks: list[Check], status: str, category: str, name: str, detail: str) -> None:
    checks.append(Check(status=status, category=category, name=name, detail=detail))


def check_file_exists(checks: list[Check], path: Path, category: str, name: str) -> None:
    if path.exists():
        add(checks, "PASS", category, name, str(path.relative_to(ROOT)))
    else:
        add(checks, "FAIL", category, name, f"Missing {path.relative_to(ROOT)}")


def collect_checks() -> list[Check]:
    checks: list[Check] = []

    if INFO_PLIST.exists():
        info = read_plist(INFO_PLIST)
        add(checks, "PASS", "plist", "Info.plist", "Found app Info.plist")
    else:
        add(checks, "FAIL", "plist", "Info.plist", "Missing app Info.plist")
        info = {}

    display_name = info.get("CFBundleDisplayName")
    if display_name == "Mapping":
        add(checks, "PASS", "metadata", "Home screen display name", "CFBundleDisplayName is Mapping")
    else:
        add(checks, "WARN", "metadata", "Home screen display name", f"Expected Mapping, found {display_name!r}")

    bundle_name = info.get("CFBundleName")
    if bundle_name:
        add(checks, "PASS", "metadata", "Bundle name", f"CFBundleName is {bundle_name}")
    else:
        add(checks, "FAIL", "metadata", "Bundle name", "CFBundleName is missing")

    for key, reason in REQUIRED_USAGE_STRINGS.items():
        value = info.get(key, "")
        if isinstance(value, str) and value.strip():
            add(checks, "PASS", "privacy", key, f"Usage string present for {reason}")
        else:
            add(checks, "FAIL", "privacy", key, f"Missing usage string for {reason}")

    temporary_location = info.get("NSLocationTemporaryUsageDescriptionDictionary", {})
    if temporary_location:
        add(checks, "PASS", "privacy", "Temporary precise location", "Temporary precise location purpose dictionary is present")
    else:
        add(checks, "WARN", "privacy", "Temporary precise location", "Confirm precise-location purpose text before review")

    iphone_orientations = info.get("UISupportedInterfaceOrientations~iphone", [])
    if iphone_orientations == ["UIInterfaceOrientationPortrait"]:
        add(checks, "PASS", "ui", "iPhone orientation", "iPhone is portrait-only")
    else:
        add(checks, "WARN", "ui", "iPhone orientation", f"Expected portrait-only, found {iphone_orientations}")

    if info.get("ITSAppUsesNonExemptEncryption") is None:
        add(checks, "WARN", "compliance", "Export compliance plist key", "Not declared; answer App Store Connect encryption questions and add the key only after final determination")
    else:
        add(checks, "PASS", "compliance", "Export compliance plist key", f"ITSAppUsesNonExemptEncryption={info.get('ITSAppUsesNonExemptEncryption')}")

    if ENTITLEMENTS.exists():
        entitlements = read_plist(ENTITLEMENTS)
        add(checks, "PASS", "entitlements", "Entitlements file", "Found MapEverything.entitlements")
    else:
        entitlements = {}
        add(checks, "FAIL", "entitlements", "Entitlements file", "Missing MapEverything.entitlements")

    if entitlements.get("com.apple.developer.networking.wifi-info") is True:
        add(checks, "PASS", "entitlements", "Wi-Fi info entitlement", "Current Wi-Fi telemetry entitlement is present")
    else:
        add(checks, "WARN", "entitlements", "Wi-Fi info entitlement", "Confirm entitlement is approved or disable current Wi-Fi telemetry for App Store release")

    check_file_exists(checks, ASSETS / "AppIcon.appiconset" / "MapEverythingAppIcon.png", "assets", "App icon")
    check_file_exists(checks, ASSETS / "MapEverythingLogo.imageset" / "MapEverythingLogo.png", "assets", "Launch logo")

    export_options = read_plist(EXPORT_OPTIONS) if EXPORT_OPTIONS.exists() else {}
    if export_options.get("method") == "app-store-connect" and export_options.get("destination") == "upload":
        add(checks, "PASS", "tools", "Export options", "App Store Connect upload export options are present")
    else:
        add(checks, "FAIL", "tools", "Export options", "tools/app-store-export-options.plist must use method=app-store-connect and destination=upload")

    for doc in REQUIRED_DOCS:
        check_file_exists(checks, ROOT / doc, "docs", doc)
    for tool in REQUIRED_TOOLS:
        check_file_exists(checks, ROOT / tool, "tools", tool)

    bundle_ids = pbx_values("PRODUCT_BUNDLE_IDENTIFIER")
    if "com.salsicha.MapEverything" in bundle_ids:
        add(checks, "PASS", "signing", "Bundle identifier", "com.salsicha.MapEverything is configured")
    else:
        add(checks, "FAIL", "signing", "Bundle identifier", f"Expected com.salsicha.MapEverything, found {bundle_ids}")

    teams = [value for value in pbx_values("DEVELOPMENT_TEAM") if value]
    if teams:
        add(checks, "PASS", "signing", "Development team", ", ".join(teams))
    else:
        add(checks, "WARN", "signing", "Development team", "No DEVELOPMENT_TEAM found; archive will need a signing team")

    signing_styles = pbx_values("CODE_SIGN_STYLE")
    if "Automatic" in signing_styles:
        add(checks, "PASS", "signing", "Code signing style", "Automatic signing is configured")
    else:
        add(checks, "WARN", "signing", "Code signing style", f"Expected Automatic signing, found {signing_styles}")

    marketing_versions = pbx_values("MARKETING_VERSION")
    build_versions = pbx_values("CURRENT_PROJECT_VERSION")
    add(checks, "PASS" if marketing_versions else "WARN", "versioning", "Marketing version", ", ".join(marketing_versions) or "Missing MARKETING_VERSION")
    add(checks, "PASS" if build_versions else "WARN", "versioning", "Build number", ", ".join(build_versions) or "Missing CURRENT_PROJECT_VERSION")

    deployment_targets = pbx_values("IPHONEOS_DEPLOYMENT_TARGET")
    if deployment_targets:
        highest_warning = any(value.startswith("26.") or value.startswith("27.") for value in deployment_targets)
        status = "WARN" if highest_warning else "PASS"
        detail = ", ".join(deployment_targets)
        if highest_warning:
            detail += " - confirm this intentionally limits App Store device reach"
        add(checks, status, "compatibility", "iOS deployment target", detail)
    else:
        add(checks, "WARN", "compatibility", "iOS deployment target", "No IPHONEOS_DEPLOYMENT_TARGET found")

    if not list(ROOT.glob("**/PrivacyInfo.xcprivacy")):
        add(checks, "WARN", "privacy", "Privacy manifest", "No PrivacyInfo.xcprivacy found; confirm whether required-reason APIs or third-party SDKs require one")

    account_side_items = [
        ("Privacy policy URL", "Add a live privacy policy URL in App Store Connect"),
        ("Privacy labels", "Declare camera/depth, location, BLE/Wi-Fi, diagnostics, local storage, and optional provider data practices"),
        ("Export compliance", "Answer App Store Connect encryption questions for networking/TLS usage"),
        ("Screenshots and previews", "Capture current portrait UI, ROS bridge panel, local bag sharing, and RViz replay"),
        ("Support URL", "Provide a support page or issue/contact URL"),
        ("Age rating", "Complete App Store Connect age rating questionnaire"),
        ("Review notes", "Explain LiDAR hardware, local ROS bridge setup, local network prompt, and optional local bag workflow"),
        ("TestFlight groups", "Create internal and external beta groups with focused mapping tasks"),
    ]
    for name, detail in account_side_items:
        add(checks, "WARN", "app-store-connect", name, detail)

    return checks


def archive_commands() -> list[str]:
    return [
        "/Applications/Xcode.app/Contents/Developer/usr/bin/xcodebuild archive "
        "-project MapEverything/MapEverything.xcodeproj "
        "-scheme MapEverything "
        "-configuration Release "
        "-destination generic/platform=iOS "
        "-archivePath build/AppStore/MapEverything.xcarchive "
        "-allowProvisioningUpdates",
        "/Applications/Xcode.app/Contents/Developer/usr/bin/xcodebuild -exportArchive "
        "-archivePath build/AppStore/MapEverything.xcarchive "
        "-exportOptionsPlist tools/app-store-export-options.plist "
        "-exportPath build/AppStore",
    ]


def print_human(checks: list[Check], include_commands: bool) -> None:
    counts = {status: sum(1 for check in checks if check.status == status) for status in ("PASS", "WARN", "FAIL")}
    print(f"App Store release readiness: {counts['PASS']} pass, {counts['WARN']} warn, {counts['FAIL']} fail")
    for check in checks:
        print(f"[{check.status}] {check.category}: {check.name} - {check.detail}")

    if include_commands:
        print("\nArchive/upload commands:")
        for command in archive_commands():
            print(command)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check MapEverything App Store release readiness.")
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    parser.add_argument("--strict", action="store_true", help="treat warnings as a non-zero result")
    parser.add_argument("--no-commands", action="store_true", help="omit archive/upload command hints in text output")
    args = parser.parse_args()

    checks = collect_checks()
    fail_count = sum(1 for check in checks if check.status == "FAIL")
    warn_count = sum(1 for check in checks if check.status == "WARN")

    if args.json:
        payload = {
            "summary": {
                "pass": sum(1 for check in checks if check.status == "PASS"),
                "warn": warn_count,
                "fail": fail_count,
            },
            "checks": [asdict(check) for check in checks],
            "archive_commands": archive_commands(),
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print_human(checks, include_commands=not args.no_commands)

    if fail_count:
        return 1
    if args.strict and warn_count:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
