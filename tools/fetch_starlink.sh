#!/usr/bin/env bash
# Fetches runs of a public Starlink one-way delay dataset and converts them
# to tuushin traces, for tuushin-sim and tuushin-netreplay:
#
#   J. Garcia, S. Sundberg and A. Brunstrom, dataset for "A detailed
#   characterization of Starlink one-way delay", ACM LEONET 2025.
#   Zenodo, doi:10.5281/zenodo.16275284, licensed CC BY 4.0.
#
# Each run is 50 s of UDP probes about 266 us apart, sent and received on
# one hardware-timestamping NIC (one clock) at an unobstructed terminal in
# Karlstad, Sweden, in April 2025, with one file for the downlink and one
# for the uplink. The data is not part of this repository; this script
# downloads the runs asked for, one member of the dataset's zip at a time
# (about 2 MB each), through Zenodo's API. Whoever publishes traces made
# from it must credit the authors as the licence requires; the converted
# files carry the credit in their comments.
#
#   tools/fetch_starlink.sh --list | head          # the runs, by start time
#   tools/fetch_starlink.sh --out traces 2025-04-11-15-16-16
#     -> traces/starlink-2025-04-11-15-16-16.dl.csv and .ul.csv
#
# A sequence number missing from a run is not necessarily a lost packet:
# the dataset's README says a send timestamp that failed to read is left
# out as well. In the runs looked at, the downlink's missing numbers come
# mostly in runs of 37 to 76 (10 to 20 ms) every 15 s, with the network's
# reconfigurations; the uplink's mostly one or two at a time, spread
# evenly. --missing lost (the default) replays every missing number as a
# loss, at a send time interpolated between its neighbours; --missing skip
# leaves them out, so that the path holds the previous packet's delay.
#
# Needs curl and unzip.

set -euo pipefail

API=https://zenodo.org/api/records/16275284/files/StarlinkOWD.zip/container
OUT=.
MISSING=lost
LIST=0
RUNS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT=$2; shift ;;
    --missing) MISSING=$2; shift ;;
    --list) LIST=1 ;;
    -*) echo "unknown option $1" >&2; exit 1 ;;
    *) RUNS+=("$1") ;;
  esac
  shift
done
case "$MISSING" in lost|skip) ;; *) echo "--missing is lost or skip" >&2; exit 1 ;; esac

if [ "$LIST" = 1 ]; then
  listing=$(curl -sSf "$API")
  grep -oE '"key": "[0-9-]+__dl\.csv\.zip"' <<< "$listing" | sed -E 's/.*"([0-9-]+)__dl.*/\1/'
  # The API lists the first 1000 members only; the later ones can still be
  # fetched by name.
  if grep -q '"truncated": true' <<< "$listing"; then
    echo "(the listing stops here; the runs go on about every 10 minutes" \
         "until 2025-04-21-15-06-16, named by their start to the second)" >&2
  fi
  exit 0
fi
[ ${#RUNS[@]} -gt 0 ] || { echo "name a run (--list shows them)" >&2; exit 1; }
mkdir -p "$OUT"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

for run in "${RUNS[@]}"; do
  for dir in dl ul; do
    member=${run}__$dir.csv.zip
    curl -sSf -o "$TMP/$member" "$API/$member"
    out="$OUT/starlink-$run.$dir.csv"
    unzip -p "$TMP/$member" | awk -F, -v missing="$MISSING" -v run="$run" -v dir="$dir" '
      BEGIN {
        print "# Starlink " (dir == "dl" ? "downlink" : "uplink") ", run " run ", Karlstad, Sweden, April 2025."
        print "# From J. Garcia, S. Sundberg and A. Brunstrom, dataset for \"A detailed"
        print "# characterization of Starlink one-way delay\", ACM LEONET 2025,"
        print "# doi:10.5281/zenodo.16275284, CC BY 4.0. Converted by tuushin"
        print "# tools/fetch_starlink.sh (send times from the run'\''s first probe;"
        print "# missing sequence numbers: " missing ")."
        print "send_ns,delay_ns"
      }
      NR == 1 { next }
      {
        seq = $1; t = $2; d = $3
        if (n == 0) { t0 = t }
        else if (missing == "lost") {
          for (k = pseq + 1; k < seq; k++) {
            printf "%d,-1\n", pt - t0 + (t - pt) * (k - pseq) / (seq - pseq)
          }
        }
        printf "%d,%d\n", t - t0, d
        pseq = seq; pt = t; n++
      }' > "$out"
    echo "$out: $(grep -c ',-1$' "$out" || true) lost of $(($(grep -cv '^#' "$out") - 1))"
  done
done
