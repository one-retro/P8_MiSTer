#!/usr/bin/env python3
"""Generate the MiSTer Downloader database for the P8 core (1Retro).

Reads a published GitHub release, hashes the files it carries, and writes the
two artifacts Downloader and users fetch:

    db.json.zip           the database itself
    downloader_p8.ini     the drop-in that points at it, copied to the card

Both land in --out, which in CI is a checkout of the orphan `db` branch, so a
run that changes nothing leaves no diff to commit.

Two rules from the database format drive the shape of this script:

  * Every URL must be immutable bytes, because the database pins an MD5 next to
    it. Release asset URLs are; anything with "latest" in it is not.
  * db_id and db_url are chosen once and never change. Changing either makes
    Downloader treat the contents as a second, unrelated database: files get
    duplicated on the SD card and cleanup stops working.

Standard library only, so the workflow needs no install step.
"""

import argparse
import hashlib
import json
import os
import re
import sys
import time
import urllib.request
import zipfile
from pathlib import Path

DB_ID = "p8"
DEFAULT_REPO = "one-retro/P8_MiSTer"

# Release assets -> where they install, relative to /media/fat. The core goes
# in the console menu folder with MiSTer's dated name (asset
# P8_YYYYMMDD.rbf -> _Console/P8_YYYYMMDD.rbf), so a release with a new date
# replaces the old file: Downloader deletes what leaves the database. The
# BIOS goes where the core loads it at start-up. Carts and saves
# (games/P8/*, saves/P8/) are never listed, so no update touches them.
CORE = re.compile(r"P8_\d{8}\.rbf")
FIXED = {"boot.rom": "games/P8/boot.rom"}


def api(url):
    request = urllib.request.Request(url, headers={"Accept": "application/vnd.github+json"})
    token = os.environ.get("GH_TOKEN") or os.environ.get("GITHUB_TOKEN")
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(request) as response:
        return json.load(response)


def release(repo, tag):
    """The release to build from: a specific tag, or the latest published one."""
    path = f"tags/{tag}" if tag else "latest"
    return api(f"https://api.github.com/repos/{repo}/releases/{path}")


def fetch(url):
    request = urllib.request.Request(url, headers={"Accept": "application/octet-stream"})
    with urllib.request.urlopen(request) as response:
        return response.read()


def install_map(rel):
    """Asset name -> install path, or fail saying what is wrong.

    Failing loudly matters more than it looks: a database that silently drops a
    file tells Downloader to delete that file from every SD card that has it.
    """
    names = [asset["name"] for asset in rel.get("assets", [])]
    cores = [n for n in names if CORE.fullmatch(n)]
    if len(cores) != 1:
        sys.exit(f"release {rel['tag_name']} needs exactly one P8_YYYYMMDD.rbf, has {cores or 'none'}")
    missing = [n for n in FIXED if n not in names]
    if missing:
        sys.exit(f"release {rel['tag_name']} is missing: {', '.join(missing)}")
    return {cores[0]: f"_Console/{cores[0]}", **FIXED}


def build_files(rel):
    assets = {asset["name"]: asset for asset in rel.get("assets", [])}
    files = {}
    for name, target in install_map(rel).items():
        data = fetch(assets[name]["browser_download_url"])
        files[target] = {
            "hash": hashlib.md5(data).hexdigest(),
            "size": len(data),
            "url": assets[name]["browser_download_url"],
        }
        print(f"{target}: {files[target]['hash']} ({files[target]['size']} bytes)")
    return files


def previous(path):
    if not path.exists():
        return None
    with zipfile.ZipFile(path) as archive:
        return json.loads(archive.read("db.json"))


def write_zip(path, name, data):
    """Write a one-entry zip whose bytes depend only on its contents.

    Zip records a mtime per entry, so without a fixed date_time every run would
    produce a different file and push a commit that changes nothing.
    """
    info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o644 << 16
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr(info, data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=os.environ.get("GITHUB_REPOSITORY", DEFAULT_REPO))
    parser.add_argument("--tag", default=None, help="release tag (default: the latest release)")
    parser.add_argument("--out", default="out", type=Path)
    args = parser.parse_args()

    db_url = f"https://raw.githubusercontent.com/{args.repo}/db/db.json.zip"
    args.out.mkdir(parents=True, exist_ok=True)
    db_zip = args.out / "db.json.zip"

    rel = release(args.repo, args.tag)
    print(f"building from {rel['tag_name']}")
    files = build_files(rel)

    # Keep the old timestamp when nothing else moved, so an unchanged database
    # stays byte-identical and the workflow has nothing to push.
    old = previous(db_zip)
    timestamp = old["timestamp"] if old and old.get("files") == files else int(time.time())

    db = {
        "v": 1,
        "db_id": DB_ID,
        "timestamp": timestamp,
        "files": files,
        "folders": {"_Console/": {}, "games/": {}, "games/P8/": {}},
    }
    write_zip(db_zip, "db.json", json.dumps(db, indent=2, sort_keys=True))

    # A plain file rather than a zip: Downloader wants the ini itself on the
    # card, so the whole install is a single wget (or a copy from a desktop).
    ini_path = args.out / f"downloader_{DB_ID}.ini"
    ini_path.write_text(f"[{DB_ID}]\ndb_url = {db_url}\ndescription = 1Retro P8 fantasy console core\n")

    print(f"wrote {db_zip} and {ini_path}")


if __name__ == "__main__":
    main()
