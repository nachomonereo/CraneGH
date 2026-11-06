import os
import re
import urllib.request
import urllib.parse
from concurrent.futures import ThreadPoolExecutor

def main():
    url = "https://www.cs.cmu.edu/~kmcrane/index.html"
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    html = urllib.request.urlopen(req, timeout=15).read().decode('utf-8', errors='ignore')
    imgs = re.findall(r'<img[^>]+src=["\']([^"\']+)["\']', html, re.I)
    
    os.makedirs("project_icons", exist_ok=True)
    tasks = []
    seen = set()
    for img in imgs:
        full_url = urllib.parse.urljoin(url, img)
        if full_url in seen:
            continue
        seen.add(full_url)
        parsed = urllib.parse.urlparse(full_url)
        parts = [p for p in parsed.path.split('/') if p]
        if 'Projects' in parts:
            idx = parts.index('Projects')
            if idx + 1 < len(parts) - 1:
                proj = parts[idx + 1]
                ext = os.path.splitext(parts[-1])[1].lower()
                if ext in ('.png', '.jpg', '.jpeg'):
                    dest = os.path.join("project_icons", f"{proj}{ext}")
                    tasks.append((proj, full_url, dest))

    print(f"Found {len(tasks)} project-specific icons on Keenan Crane's page.")
    
    def dl(t):
        proj, u, dest = t
        if os.path.exists(dest) and os.path.getsize(dest) > 100:
            return f"[EXISTS] {proj}"
        try:
            r = urllib.request.Request(u, headers={'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(r, timeout=15) as resp:
                data = resp.read()
                with open(dest, 'wb') as f:
                    f.write(data)
            return f"[OK] {proj} -> {os.path.basename(dest)} ({len(data)//1024} KB)"
        except Exception as e:
            return f"[FAIL] {proj}: {e}"

    with ThreadPoolExecutor(max_workers=10) as ex:
        for res in ex.map(dl, tasks):
            print(res)

if __name__ == "__main__":
    main()
