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
    
    os.makedirs("project_images", exist_ok=True)
    tasks = []
    seen = set()
    for img in imgs:
        full_url = urllib.parse.urljoin(url, img)
        if full_url in seen:
            continue
        seen.add(full_url)
        fname = os.path.basename(urllib.parse.urlparse(full_url).path)
        if not fname:
            continue
        dest = os.path.join("project_images", fname)
        tasks.append((fname, full_url, dest))
        
    print(f"Found {len(tasks)} unique images on index.html.")
    
    def dl(t):
        fname, u, dest = t
        if os.path.exists(dest) and os.path.getsize(dest) > 100:
            return f"[EXISTS] {fname}"
        try:
            r = urllib.request.Request(u, headers={'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(r, timeout=15) as resp:
                data = resp.read()
                with open(dest, 'wb') as f:
                    f.write(data)
            return f"[OK] {fname} ({len(data)//1024} KB)"
        except Exception as e:
            return f"[FAIL] {fname}: {e}"

    with ThreadPoolExecutor(max_workers=10) as ex:
        for res in ex.map(dl, tasks):
            print(res)

if __name__ == "__main__":
    main()
