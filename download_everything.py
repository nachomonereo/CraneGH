import os
import sys
import urllib.request
import urllib.parse
import zipfile
from concurrent.futures import ThreadPoolExecutor, as_completed

sys.stdout.reconfigure(encoding='utf-8')

def fetch_file(task):
    category, project, filename, url, dest_path = task
    if os.path.exists(dest_path) and os.path.getsize(dest_path) > 500:
        return (True, f"[EXISTS] {category}/{project}/{filename} ({os.path.getsize(dest_path)//1024} KB)")
    
    os.makedirs(os.path.dirname(dest_path), exist_ok=True)
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'})
    try:
        with urllib.request.urlopen(req, timeout=20) as resp:
            data = resp.read()
            with open(dest_path, 'wb') as f_out:
                f_out.write(data)
            return (True, f"[OK] {category}/{project}/{filename} ({len(data)//1024} KB)")
    except Exception as e:
        return (False, f"[FAIL] {category}/{project}/{filename} <- {url} : {e}")

def get_project_and_name(url, index):
    parsed = urllib.parse.urlparse(url)
    path_parts = [p for p in parsed.path.split('/') if p]
    filename = path_parts[-1] if path_parts else f"file_{index}.pdf"
    
    if 'Projects' in path_parts:
        idx = path_parts.index('Projects')
        if idx + 1 < len(path_parts) - 1:
            project = path_parts[idx + 1]
        else:
            project = "RootProjects"
    elif 'kmcrane' in parsed.netloc or 'cmu.edu' in parsed.netloc:
        project = "CMU_Other"
    else:
        # External domain
        domain = parsed.netloc.replace('www.', '').split('.')[0]
        project = f"External_{domain}"
        # Prefix filename to avoid collision in External
        if filename.lower() in ('paper.pdf', 'index.pdf', 'thesis.pdf', 'dissertation.pdf'):
            sub = "_".join(path_parts[:-1])[-30:]
            sub = "".join(c if c.isalnum() else "_" for c in sub)
            filename = f"{sub}_{filename}"
            
    return project, filename

def main():
    tasks = []
    seen_urls = set()

    # 1. All PDFs from pdf_catalog.txt
    if os.path.exists("pdf_catalog.txt"):
        with open("pdf_catalog.txt", "r", encoding="utf-8") as f:
            lines = [line.strip().split('\t') for line in f if line.strip()]
        
        cv_added = False
        for i, parts in enumerate(lines):
            url = parts[0]
            # Normalize http to https for cs.cmu.edu to avoid duplicate downloads
            if url.startswith("http://www.cs.cmu.edu/"):
                url = url.replace("http://", "https://", 1)
            if url in seen_urls:
                continue
            if "KeenanCraneCV.pdf" in url:
                if cv_added:
                    continue
                cv_added = True
                seen_urls.add(url)
                dest = os.path.abspath(os.path.join("papers", "AuthorCV", "KeenanCraneCV.pdf"))
                tasks.append(("papers", "AuthorCV", "KeenanCraneCV.pdf", url, dest))
                continue
                
            seen_urls.add(url)
            project, filename = get_project_and_name(url, i)
            dest = os.path.abspath(os.path.join("papers", project, filename))
            # Avoid collision if same project has two files with identical name
            if any(t[4] == dest for t in tasks):
                filename = f"{i}_{filename}"
                dest = os.path.abspath(os.path.join("papers", project, filename))
            tasks.append(("papers", project, filename, url, dest))

    # 2. Source Code Archives & Direct Source Files from code_catalog.txt
    source_Targets = [
        "HeatMethod/HeatMethodSource.zip", # Unity C#
        "HeatMethod/code.zip",             # ANSI C
        "SpinTransformations/spinxform_commandline.zip",
        "SpinTransformations/spinxform_fast.zip",
        "StripePatterns/StripePatterns.zip",
        "StripePatterns/code.zip",
        "SubgridMarching/SubgridMarchingTetrahedron-C.zip",
        "TrivialConnections/TrivialConnectionsMATLAB.zip",
        "TrivialConnections/TrivialConnectionsSource.zip",
        "MonteCarloGeometryProcessing/WoSLaplace2D.cpp.html",
        "MonteCarloGeometryProcessing/WoSPoisson2D.cpp.html"
    ]
    for st in source_Targets:
        url = f"https://www.cs.cmu.edu/~kmcrane/Projects/{st}"
        proj, fname = st.split('/')[0], st.split('/')[1]
        dest = os.path.abspath(os.path.join("code_reference", proj, fname))
        tasks.append(("code", proj, fname, url, dest))

    print(f"Starting parallel download of {len(tasks)} items (PDFs + Source Code)...", flush=True)
    
    success_count = 0
    fail_count = 0
    with ThreadPoolExecutor(max_workers=10) as executor:
        future_to_task = {executor.submit(fetch_file, t): t for t in tasks}
        for idx, future in enumerate(as_completed(future_to_task), 1):
            ok, msg = future.result()
            if ok:
                success_count += 1
            else:
                fail_count += 1
            print(f"[{idx}/{len(tasks)}] {msg}", flush=True)

    print(f"\nCompleted! Success: {success_count}, Failed: {fail_count}", flush=True)

    # Extract downloaded source code zips
    code_dir = os.path.abspath("code_reference")
    if os.path.exists(code_dir):
        for root, dirs, files in os.walk(code_dir):
            for file in files:
                if file.endswith(".zip"):
                    zpath = os.path.join(root, file)
                    extract_dir = os.path.join(root, file[:-4])
                    if not os.path.exists(extract_dir):
                        try:
                            with zipfile.ZipFile(zpath, 'r') as zf:
                                zf.extractall(extract_dir)
                            print(f"Extracted {zpath} -> {extract_dir}", flush=True)
                        except Exception as e:
                            print(f"Failed to extract {zpath}: {e}", flush=True)

if __name__ == "__main__":
    main()
