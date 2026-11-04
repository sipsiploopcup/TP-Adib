"""Cas synthétiques : aucun de ces points n'est une acquisition réelle."""

import csv

from acquisition import ScanReader, save_scan


def test_complete_scan(tmp_path):
    reader = ScanReader()
    for line in ("N,Angle,Distance,Intensity,x,y", "0,359,1000,200,999,17",
                 "1,180,2000,201,-2000,0", "2,1,1000,202,999,-17"):
        assert reader.feed(line) is None
    points = reader.feed("")
    assert len(points) == 3
    path = save_scan(points, tmp_path)
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["dist_mm"] == "1000"
    assert rows[2]["y_lib_mm"] == "-17"


def test_reject_partial_missing_or_corrupt_scan():
    for lines in (("1,180,100,200,0,100",),
                  ("0,359,100,200,100,0", "2,1,100,200,100,0"),
                  ("0,359,100,200,100,0", "garbage", "1,1,100,200,100,0"),
                  ("0,nan,100,200,100,0", "1,1,100,200,100,0"),
                  ("0,359,100,200,100,0", "1,250,100,200,0,-100")):
        reader = ScanReader()
        for line in lines:
            reader.feed(line)
        assert reader.feed("") is None


def test_recover_at_next_index_zero():
    reader = ScanReader()
    for line in (">lidar:1:2;|xy", "4,45,100,200,70,70",
                 "0,359,100,200,100,0", "1,1,100,200,100,0"):
        reader.feed(line)
    assert len(reader.feed("")) == 2
