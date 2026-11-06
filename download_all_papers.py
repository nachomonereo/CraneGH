import os
import urllib.request
import time

def download_papers():
    with open('pdf_catalog.txt', 'r', encoding='utf-8') as f:
        lines = [line.strip().split('\t') for line in f if line.strip()]

    # Filter out CVs, generic notes
    targets = []
    seen_urls = set()
    
    # Priority projects for Keenan Crane's DDG tools
    priority_topics = [
        'HeatMethod', 'VectorHeatMethod', 'BoundaryFirstFlattening',
        'GloballyOptimalDirectionFields', 'TrivialConnections', 'StripePatterns',
        'RepulsiveCurves', 'RepulsiveSurfaces', 'RepulsiveShells',
        'NavigatingIntrinsicTriangulations', 'CEPS', 'FlipOut',
        'DiscreteDevelopable', 'AuxeticDesign', 'DeployableAuxetics',
        'NonmanifoldLaplace', 'WalkOnStars', 'MonteCarloGeometryProcessing',
        'SpinTransformations', 'ConformalWillmoreFlow', 'OptimalCones',
        'WindingNumbersOnSurfaces', 'RobustPlanarMaps', 'VariationalCuts',
        'DDG'
    ]

    for url, title in lines:
        if 'KeenanCraneCV.pdf' in url:
            continue
        if url in seen_urls:
            continue
        seen_urls.add(url)
        
        # Check topic
        topic = 'General'
        for pt in priority_topics:
            if pt.lower() in url.lower():
                topic = pt
                break
                
        targets.append((topic, url))

    out_dir = os.path.abspath('papers')
    os.makedirs(out_dir, exist_ok=True)
    print(f"Total papers to download: {len(targets)}")

    for idx, (topic, url) in enumerate(targets, 1):
        filename = url.split('/')[-1]
        if not filename.endswith('.pdf'):
            filename = f"paper_{idx}.pdf"
            
        topic_dir = os.path.join(out_dir, topic)
        os.makedirs(topic_dir, exist_ok=True)
        local_path = os.path.join(topic_dir, filename)

        if os.path.exists(local_path) and os.path.getsize(local_path) > 1000:
            print(f"[{idx}/{len(targets)}] Already exists: {topic}/{filename}")
            continue

        print(f"[{idx}/{len(targets)}] Downloading {topic}/{filename} from {url}...")
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        try:
            with urllib.request.urlopen(req, timeout=25) as resp:
                data = resp.read()
                with open(local_path, 'wb') as f_out:
                    f_out.write(data)
                print(f"    Saved ({len(data) // 1024} KB)")
        except Exception as e:
            print(f"    Failed: {e}")
        time.sleep(0.3)

if __name__ == '__main__':
    download_papers()
