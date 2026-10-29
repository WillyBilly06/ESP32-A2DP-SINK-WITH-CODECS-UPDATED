"""Open the authorized host checkpoint PR using Git's existing authentication."""
import json
import os
from pathlib import Path
import subprocess
import urllib.error
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = "WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED"
BRANCH = "codex/native-host-runtime-20261028"


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, *args, **kwargs):
        raise RuntimeError("Unexpected API redirect")


def main():
    destination = ROOT / "_build/host/publication.json"
    payload = json.loads((ROOT / "_build/host/pr_request.json").read_text(encoding="utf-8-sig"))
    if payload["head"] != BRANCH or payload["base"] != "main" or payload["draft"] is not True:
        raise RuntimeError("Unexpected publication target")
    environment = dict(os.environ, GIT_TERMINAL_PROMPT="0", GCM_INTERACTIVE="never")
    result = subprocess.run(["git", "-c", "credential.interactive=never", "credential", "fill"],
        input="protocol=https\nhost=github.com\nusername=WillyBilly06\n\n", text=True,
        capture_output=True, env=environment, timeout=20, cwd=ROOT.parent)
    if result.returncode:
        raise RuntimeError("Existing noninteractive GitHub authentication is unavailable")
    credential = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    if credential.get("username", "").lower() != "willybilly06" or not credential.get("password"):
        raise RuntimeError("Git authentication does not match the authorized repository owner")
    opener = urllib.request.build_opener(NoRedirect())

    def api(path, data=None, method=None):
        request = urllib.request.Request("https://api.github.com/repos/" + REPOSITORY + path,
            data=None if data is None else json.dumps(data).encode(), method=method, headers={
                "Authorization": "Bearer " + credential["password"],
                "Accept": "application/vnd.github+json", "X-GitHub-Api-Version": "2022-11-28",
                "Content-Type": "application/json", "User-Agent": "native-bt-host-checkpoint"})
        try:
            with opener.open(request, timeout=20) as response:
                return json.loads(response.read(500000))
        except urllib.error.HTTPError as error:
            raise RuntimeError("GitHub publication API returned HTTP " + str(error.code)) from None

    query = urllib.parse.urlencode({"state": "open", "head": "WillyBilly06:" + BRANCH, "base": "main"})
    existing = api("/pulls?" + query)
    if len(existing) > 1:
        raise RuntimeError("Multiple matching pull requests require review")
    if existing:
        pull = existing[0]
        if not pull['draft']:
            raise RuntimeError('Draft state changed; preserve the review workflow')
        if pull['title'] != payload['title'] or pull['body'] != payload['body']:
            pull = api('/pulls/'+str(pull['number']), {'title':payload['title'],'body':payload['body']}, method='PATCH')
    else:
        pull = api("/pulls", payload)
    report = {"repository": REPOSITORY, "branch": BRANCH, "commit": pull["head"]["sha"],
              "pull_request": pull["html_url"], "number": pull["number"], "draft": pull["draft"]}
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    credential.clear()
    result = None
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
