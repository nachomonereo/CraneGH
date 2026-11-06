import os
import re
import urllib.request
import urllib.parse
from html.parser import HTMLParser

class LinkExtractor(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []
        self.current_tag = None
        self.current_attrs = {}
        self.current_text = []

    def handle_starttag(self, tag, attrs):
        self.current_tag = tag
        self.current_attrs = dict(attrs)
        self.current_text = []

    def handle_endtag(self, tag):
        if tag == 'a' and 'href' in self.current_attrs:
            text = " ".join("".join(self.current_text).split())
            self.links.append((self.current_attrs['href'], text))
        self.current_tag = None
        self.current_attrs = {}
        self.current_text = []

    def handle_data(self, data):
        self.current_text.append(data)

def get_page(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'})
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return resp.read().decode('utf-8', errors='ignore')
    except Exception as e:
        print(f"Error fetching {url}: {e}")
        return ""

def main():
    root_url = "https://www.cs.cmu.edu/~kmcrane/"
    pages_to_crawl = [
        "https://www.cs.cmu.edu/~kmcrane/index.html",
        "https://www.cs.cmu.edu/~kmcrane/Projects/index.html"
    ]
    
    seen_pages = set(pages_to_crawl)
    queue = list(pages_to_crawl)
    all_pdfs = {}
    
    # Crawl main pages and project subpages
    while queue:
        curr_url = queue.pop(0)
        print(f"Fetching: {curr_url}")
        html = get_page(curr_url)
        if not html:
            continue
            
        parser = LinkExtractor()
        try:
            parser.feed(html)
        except Exception as e:
            print(f"Parse error on {curr_url}: {e}")
            continue
            
        for href, text in parser.links:
            full_url = urllib.parse.urljoin(curr_url, href)
            # Remove hash
            full_url = urllib.parse.urldefrag(full_url)[0]
            
            # Check if PDF
            if full_url.lower().endswith('.pdf'):
                if full_url not in all_pdfs:
                    title = text if text else os.path.basename(full_url)
                    all_pdfs[full_url] = title
            elif full_url.startswith(root_url) and ('Projects/' in full_url or 'Publications/' in full_url):
                if full_url.endswith('.html') or full_url.endswith('/'):
                    if full_url not in seen_pages and len(seen_pages) < 80:
                        seen_pages.add(full_url)
                        queue.append(full_url)
                        
    print(f"\nFound {len(all_pdfs)} PDFs total.")
    
    # Save list to file
    with open("pdf_catalog.txt", "w", encoding="utf-8") as f:
        for url, title in sorted(all_pdfs.items()):
            f.write(f"{url}\t{title}\n")
            print(f"- {title} -> {url}")

if __name__ == "__main__":
    main()
