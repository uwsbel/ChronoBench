import argparse
import json
import time
from urllib.error import URLError
from urllib.request import Request, urlopen


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Wait for an OpenAI-compatible server to expose /models.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8000/v1")
    parser.add_argument("--timeout", type=float, default=1800.0)
    parser.add_argument("--interval", type=float, default=5.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    url = args.base_url.rstrip("/") + "/models"
    deadline = time.time() + args.timeout
    last_error = None

    while time.time() < deadline:
        try:
            request = Request(url, headers={"Authorization": "Bearer 0"})
            with urlopen(request, timeout=args.interval) as response:
                payload = json.loads(response.read().decode("utf-8"))
            model_ids = [item.get("id", "<unknown>") for item in payload.get("data", [])]
            print(f"OpenAI endpoint is ready: {', '.join(model_ids) if model_ids else url}")
            return
        except (OSError, URLError, json.JSONDecodeError) as exc:
            last_error = exc
            time.sleep(args.interval)

    raise SystemExit(f"Timed out waiting for {url}. Last error: {last_error}")


if __name__ == "__main__":
    main()
