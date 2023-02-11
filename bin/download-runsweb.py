"""
Not to be run directly.  Use runsweb.sh which ensures you have GH_TOKEN set
"""

import json
import os
from urllib.request import Request, urlopen

# TODO(jayen): after we have non-pre-release releases, use /releases/latest
releases_url = 'https://api.github.com/repos/UNSWComputing/rUNSWeb/releases'

releases_request = Request(releases_url, headers={'Authorization': f'token {os.environ["GH_TOKEN"]}'})
with urlopen(releases_request) as releases_stream:
    releases = json.load(releases_stream)
latest_release = releases[0]
latest_assets = latest_release['assets']
appimage_asset = next(asset for asset in latest_assets if 'AppImage' in asset['name'])
appimage_url = appimage_asset['url']
appimage_size = appimage_asset['size']

print(f'Downloading from {appimage_url}')
appimage_request = Request(appimage_url, headers={'Accept': 'application/octet-stream'})
appimage_request.add_unredirected_header('Authorization', f'token {os.environ["GH_TOKEN"]}')
num_bytes_read = 0
with urlopen(appimage_request) as appimage_stream:
    with open(f'{os.environ["RUNSWIFT_CHECKOUT_DIR"]}/softwares/rUNSWeb.AppImage', "wb") as appimage_file:
        while num_bytes_read < appimage_size:
            chunk = appimage_stream.read(2 ** 15)
            appimage_file.write(chunk)
            num_bytes_read += len(chunk)
            print(f'{round(100 * num_bytes_read / appimage_size)}% of {appimage_size} bytes: {num_bytes_read}',
                  end='\r',
                  flush=True)
        print('\n')
