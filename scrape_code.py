import os
import urllib.request
import urllib.parse
from html.parser import HTMLParser

class CodeLinkExtractor(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []
        self.current_attrs = {}
        self.current_text = []

    def handle_starttag(self, tag, attrs):
        if tag == 'a':
            self.current_attrs = dict(attrs)
            self.current_text = []

    def handle_endtag(self, tag):
        if tag == 'a' and 'href' in self.current_attrs:
            text = " ".join("".join(self.current_text).split())
            self.links.append((self.current_attrs['href'], text))
            self.current_attrs = {}
            self.current_text = []

    def handle_data(self, data):
        if self.current_attrs:
            self.current_text.append(data)

def get_page(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return resp.read().decode('utf-8', errors='ignore')
    except Exception:
        return ""

def main():
    root_url = "https://www.cs.cmu.edu/~kmcrane/"
    pages_to_crawl = [
        "https://www.cs.cmu.edu/~kmcrane/index.html",
        "https://www.cs.cmu.edu/~kmcrane/Projects/index.html"
    ]
    
    seen_pages = set(pages_to_crawl)
    queue = list(pages_to_crawl)
    
    code_archives = {}
    github_repos = {}
    source_files = {}

    code_exts = ('.zip', '.tar.gz', '.tgz', '.rar', '.7z', '.cpp', '.c', '.h', '.hpp', '.py', '.m', '.js', '.cs', '.glsl', '.frag', '.vert')

    while queue:
        curr_url = queue.pop(0)
        html = get_page(curr_url)
        if not html:
            continue
            
        parser = CodeLinkExtractor()
        try:
            parser.feed(html)
        except Exception:
            continue
            
        for href, text in parser.links:
            full_url = urllib.parse.urljoin(curr_url, href)
            full_url = urllib.parse.urldefrag(full_url)[0]
            lower_url = full_url.lower()

            # Check GitHub / GitLab / Bitbucket
            if any(host in lower_url for host in ['github.com', 'gitlab.com', 'bitbucket.org']):
                github_repos[full_url] = (curr_url, text)
            elif lower_url.endswith(code_exts) or any( lower_url.endswith(ext + '.html') for ext in ('.cpp', '.h', '.c') ):
                if lower_url.endswith(('.zip', '.tar.gz', '.tgz', '.rar', '.7z')):
                    code_archives[full_url] = (curr_url, text)
                else:
                    source_files[full_url] = (curr_url, text)
            elif full_url.startswith(root_url) and ('Projects/' in full_url or 'ddg' in lower_url):
                if full_url.endswith('.html') or full_url.endswith('/'):
                    if full_url not in seen_pages and len(seen_pages) < 100:
                        seen_pages.add(full_url)
                        queue.append(full_url)

    with open("code_catalog.txt", "w", encoding="utf-8") as f:
        f.write("=== GITHUB / GIT REPOS ===\n")
        for url, (src, txt) in sorted(github_repos.items()):
            f.write(f"{url}\t{txt}\t{src}\n")
        f.write("\n=== CODE ARCHIVES (ZIP/TGZ) ===\n")
        for url, (src, txt) in sorted(code_archives.items()):
            f.write(f"{url}\t{txt}\t{src}\n")
        f.write("\n=== DIRECT SOURCE FILES ===\n")
        for url, (src, txt) in sorted(source_files.items()):
            f.write(f"{url}\t{txt}\t{src}\n")

    print(f"Found {len(github_repos)} Git repos, {len(code_archives)} Code Archives, and {len(source_files)} direct source files.")
    print("\n--- GitHub Repos ---")
    for url, (src, txt) in sorted(github_repos.items()):
        proj = src.replace(root_url, '')
        print(f"[{proj}] {txt}: {url}")

    print("\n--- Code Archives ---")
    for url, (src, txt) in sorted(code_archives.items()):
        proj = src.replace(root_url, '')
        print(f"[{proj}] {txt}: {url}")

    print("\n--- Direct Source Files ---")
    for url, (src, txt) in sorted(source_files.items()):
        proj = src.replace(root_url, '')
        print(f"[{proj}] {txt}: {url}")

if __name__ == "__main__":
    main()
