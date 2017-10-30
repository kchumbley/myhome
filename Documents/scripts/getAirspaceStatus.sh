#!/bin/bash

curl -X "GET" "https://api.airmap.com/status/v2/point/?latitude=47.550689&longitude=-117.599550&weather=true&types=airport,controlled_airspace,special_use_airspace,school,tfr" -H "X-API-Key: " | python -m json.tool | pygmentize -l javascript
