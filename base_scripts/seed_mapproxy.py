#!/usr/bin/env python3
"""Pre-download (seed) MapProxy tiles around the origins in mapviz_origins.yaml.

Builds a mapproxy-seed config for a box of the requested size centered on one
(or all) of the mapviz origins, copies it into the running MapProxy container,
and runs `mapproxy-seed` there.  Tiles land in the container's cache directory
(~/mapproxy/cache_data on the host) so they are available offline afterwards.

Examples:
    # List the origins that can be seeded
    bash base_scripts/seed_mapproxy.py --list

    # Interactive pick, default 2 km box, zoom 10-19
    bash base_scripts/seed_mapproxy.py

    # One site, 4 km box
    ./base_scripts/seed_mapproxy.py kahana --size 4000

    # Rectangle, higher zoom only
    ./base_scripts/seed_mapproxy.py punaluu --width 6000 --height 3000 --levels 14-19

    # Every origin in the file, no confirmation prompt
    ./base_scripts/seed_mapproxy.py --all --yes
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
import tempfile

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ORIGINS = os.path.join(REPO_ROOT, 'config', 'mapviz_origins.yaml')
DEFAULT_CONTAINER = 'mapproxy-ct'
DEFAULT_MAPPROXY_YAML = '/mapproxy/mapproxy.yaml'
SEED_YAML_IN_CONTAINER = '/tmp/mapviz_seed.yaml'

# Rough tile-count above which we nag the user before hammering the tile server
TILE_WARN_THRESHOLD = 50000

EARTH_RADIUS = 6378137.0
METERS_PER_DEG_LAT = 111320.0


def parse_args():
    p = argparse.ArgumentParser(
        description='Seed MapProxy tiles around mapviz origins.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('Examples:')[1],
    )
    p.add_argument('origin', nargs='*',
                   help='origin name(s) to seed; omit for an interactive picker')
    p.add_argument('-a', '--all', action='store_true',
                   help='seed every origin in the file')
    p.add_argument('-l', '--list', action='store_true',
                   help='list the available origins and exit')
    p.add_argument('-f', '--origins-file', default=DEFAULT_ORIGINS,
                   help=f'mapviz origins yaml (default: {DEFAULT_ORIGINS})')
    p.add_argument('-s', '--size', type=float, default=2000.0,
                   help='side length in meters of the square box around each '
                        'origin (default: 2000)')
    p.add_argument('-W', '--width', type=float,
                   help='east-west box size in meters (overrides --size)')
    p.add_argument('-H', '--height', type=float,
                   help='north-south box size in meters (overrides --size)')
    p.add_argument('-z', '--levels', default='10-19',
                   help='zoom levels, "MIN-MAX" or a comma list (default: 10-19)')
    p.add_argument('-c', '--concurrency', type=int, default=4,
                   help='parallel tile downloads (default: 4)')
    p.add_argument('--container', default=DEFAULT_CONTAINER,
                   help=f'MapProxy container name (default: {DEFAULT_CONTAINER})')
    p.add_argument('--mapproxy-yaml', default=DEFAULT_MAPPROXY_YAML,
                   help='path to mapproxy.yaml *inside* the container '
                        f'(default: {DEFAULT_MAPPROXY_YAML})')
    p.add_argument('--cache', help='cache name to seed (default: every cache '
                                   'found in mapproxy.yaml)')
    p.add_argument('-n', '--dry-run', action='store_true',
                   help='print the generated seed config and tile estimate, '
                        'then exit without downloading')
    p.add_argument('-y', '--yes', action='store_true',
                   help='skip the confirmation prompt')
    return p.parse_args()


def load_origins(path):
    if not os.path.exists(path):
        sys.exit(f'error: origins file not found: {path}')
    with open(path) as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, list):
        sys.exit(f'error: expected a list of origins in {path}')
    origins = {}
    for entry in data:
        try:
            origins[entry['name']] = (float(entry['latitude']),
                                      float(entry['longitude']))
        except (KeyError, TypeError, ValueError):
            sys.exit(f'error: malformed origin entry in {path}: {entry!r}')
    return origins


def parse_levels(spec):
    """Return a sorted list of zoom levels from "10-19" or "12,14,16"."""
    levels = set()
    for chunk in spec.split(','):
        chunk = chunk.strip()
        if not chunk:
            continue
        if '-' in chunk:
            lo, hi = chunk.split('-', 1)
            levels.update(range(int(lo), int(hi) + 1))
        else:
            levels.add(int(chunk))
    if not levels:
        sys.exit(f'error: could not parse levels: {spec!r}')
    if min(levels) < 0:
        sys.exit('error: zoom levels must be >= 0')
    return sorted(levels)


def bbox_around(lat, lon, width_m, height_m):
    """Lon/lat bounding box (EPSG:4326) of a width x height box about a point."""
    half_lat = (height_m / 2.0) / METERS_PER_DEG_LAT
    # Guard against cos() blowing up near the poles
    cos_lat = max(math.cos(math.radians(lat)), 1e-6)
    half_lon = (width_m / 2.0) / (METERS_PER_DEG_LAT * cos_lat)
    return [
        round(lon - half_lon, 8),  # west
        round(max(lat - half_lat, -85.05), 8),  # south
        round(lon + half_lon, 8),  # east
        round(min(lat + half_lat, 85.05), 8),  # north
    ]


def tile_xy(lat, lon, zoom):
    """Standard web-mercator (XYZ / GLOBAL_MERCATOR, origin ul) tile indices."""
    n = 2 ** zoom
    lat = max(min(lat, 85.05), -85.05)
    x = int((lon + 180.0) / 360.0 * n)
    lat_rad = math.radians(lat)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return max(0, min(x, n - 1)), max(0, min(y, n - 1))


def estimate_tiles(bbox, levels):
    west, south, east, north = bbox
    total = 0
    for z in levels:
        x0, y0 = tile_xy(north, west, z)   # top-left
        x1, y1 = tile_xy(south, east, z)   # bottom-right
        total += (abs(x1 - x0) + 1) * (abs(y1 - y0) + 1)
    return total


def docker_exec(container, cmd, **kwargs):
    return subprocess.run(['docker', 'exec', container] + cmd, **kwargs)


def read_container_mapproxy_cfg(container, path):
    """Read mapproxy.yaml out of the container so we can auto-detect caches."""
    proc = docker_exec(container, ['cat', path],
                       capture_output=True, text=True)
    if proc.returncode != 0:
        sys.exit(f'error: could not read {path} from container '
                 f'{container}:\n{proc.stderr.strip()}')
    try:
        return yaml.safe_load(proc.stdout) or {}
    except yaml.YAMLError as exc:
        sys.exit(f'error: {path} in {container} is not valid yaml: {exc}')


def container_running(container):
    proc = subprocess.run(
        ['docker', 'ps', '--filter', f'name=^{container}$', '--format', '{{.Names}}'],
        capture_output=True, text=True)
    return container in proc.stdout.split()


def pick_origins_interactively(origins):
    names = list(origins)
    print('Available origins:')
    for i, name in enumerate(names, 1):
        lat, lon = origins[name]
        print(f'  {i:2d}) {name:<16} {lat:>11.6f}, {lon:>12.6f}')
    print(f'  {len(names) + 1:2d}) all of the above')
    try:
        choice = input('Select origin [number or name]: ').strip()
    except (EOFError, KeyboardInterrupt):
        sys.exit('\naborted')
    if choice.isdigit():
        idx = int(choice)
        if idx == len(names) + 1:
            return names
        if 1 <= idx <= len(names):
            return [names[idx - 1]]
        sys.exit(f'error: {idx} is not a valid selection')
    if choice in origins:
        return [choice]
    sys.exit(f'error: unknown origin {choice!r}')


def main():
    # Keep our output interleaved correctly with mapproxy-seed's when piped
    sys.stdout.reconfigure(line_buffering=True)
    args = parse_args()
    origins = load_origins(args.origins_file)

    if args.list:
        for name, (lat, lon) in origins.items():
            print(f'{name:<16} {lat:>11.6f}, {lon:>12.6f}')
        return 0

    if args.all:
        selected = list(origins)
    elif args.origin:
        unknown = [n for n in args.origin if n not in origins]
        if unknown:
            sys.exit(f'error: unknown origin(s): {", ".join(unknown)}\n'
                     f'known: {", ".join(origins)}')
        selected = args.origin
    else:
        selected = pick_origins_interactively(origins)

    width = args.width if args.width else args.size
    height = args.height if args.height else args.size
    levels = parse_levels(args.levels)

    if shutil.which('docker') is None:
        sys.exit('error: docker not found on PATH')
    if not container_running(args.container):
        sys.exit(f'error: container {args.container!r} is not running.\n'
                 f'       start it with: cd docker/base_station && docker compose up -d')

    cfg = read_container_mapproxy_cfg(args.container, args.mapproxy_yaml)
    all_caches = cfg.get('caches') or {}
    if not all_caches:
        sys.exit(f'error: no caches defined in {args.mapproxy_yaml}')
    if args.cache:
        if args.cache not in all_caches:
            sys.exit(f'error: cache {args.cache!r} not in {args.mapproxy_yaml}; '
                     f'available: {", ".join(all_caches)}')
        cache_names = [args.cache]
    else:
        cache_names = list(all_caches)

    # Levels beyond the grid's depth are silently skipped by mapproxy-seed, so
    # warn instead of letting the user think they got zoom 21 imagery.
    grid_defs = cfg.get('grids') or {}
    max_level = 0
    for cache_name in cache_names:
        for grid_name in all_caches[cache_name].get('grids', []):
            grid = grid_defs.get(grid_name, {})
            # GLOBAL_MERCATOR and friends default to 20 levels (0-19)
            max_level = max(max_level, int(grid.get('num_levels', 20)) - 1)
    if max_level and max(levels) > max_level:
        print(f'warning: grid only goes to zoom {max_level}; levels above that '
              f'will be skipped.\n'
              f'         add "num_levels: {max(levels) + 1}" to the grid in '
              f'mapproxy.yaml to go deeper.', file=sys.stderr)

    seeds, coverages = {}, {}
    grand_total = 0
    print(f'Seeding {width:.0f} m x {height:.0f} m, zoom '
          f'{min(levels)}-{max(levels)}, cache(s): {", ".join(cache_names)}\n')
    for name in selected:
        lat, lon = origins[name]
        bbox = bbox_around(lat, lon, width, height)
        count = estimate_tiles(bbox, levels)
        grand_total += count * len(cache_names)
        coverages[f'{name}_box'] = {'bbox': bbox, 'srs': 'EPSG:4326'}
        seeds[name] = {
            # fresh list per seed so yaml.safe_dump doesn't emit anchors
            'caches': list(cache_names),
            'coverages': [f'{name}_box'],
            'levels': {'from': min(levels), 'to': max(levels)},
        }
        print(f'  {name:<16} {lat:>11.6f}, {lon:>12.6f}  '
              f'bbox [{bbox[0]:.5f}, {bbox[1]:.5f}, {bbox[2]:.5f}, {bbox[3]:.5f}]  '
              f'~{count:,} tiles')

    seed_cfg = {'seeds': seeds, 'coverages': coverages}
    print(f'\nEstimated total: ~{grand_total:,} tiles '
          f'(already-cached tiles are skipped)')

    if args.dry_run:
        print('\n--- seed config (dry run, nothing downloaded) ---')
        print(yaml.safe_dump(seed_cfg, sort_keys=False, default_flow_style=False))
        return 0

    if grand_total > TILE_WARN_THRESHOLD:
        print(f'warning: that is a lot of tiles. Consider a smaller --size or a '
              f'shallower --levels.', file=sys.stderr)
    if not args.yes:
        try:
            if input('Proceed? [y/N] ').strip().lower() not in ('y', 'yes'):
                print('aborted')
                return 1
        except (EOFError, KeyboardInterrupt):
            print('\naborted')
            return 1

    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as fh:
        yaml.safe_dump(seed_cfg, fh, sort_keys=False, default_flow_style=False)
        local_seed = fh.name
    try:
        cp = subprocess.run(
            ['docker', 'cp', local_seed,
             f'{args.container}:{SEED_YAML_IN_CONTAINER}'],
            capture_output=True, text=True)
        if cp.returncode != 0:
            sys.exit(f'error: docker cp failed:\n{cp.stderr.strip()}')

        cmd = ['docker', 'exec']
        if sys.stdout.isatty():
            cmd.append('-t')
        cmd += [args.container, 'mapproxy-seed',
                '-f', args.mapproxy_yaml,
                '-s', SEED_YAML_IN_CONTAINER,
                '-c', str(args.concurrency),
                '--seed', ','.join(selected)]
        print(f'\n$ {" ".join(cmd)}\n')
        rc = subprocess.run(cmd).returncode
    finally:
        os.unlink(local_seed)

    if rc == 0:
        print('\nDone. Tiles are cached on the host in ~/mapproxy/cache_data.')
    return rc


if __name__ == '__main__':
    sys.exit(main())
