import requests
import json

def post_to_express(time, pv, inverter, wind, grid, load):
    URL = "https://damp-gorge-19491.herokuapp.com"
    URL += "/tm4cInput"

    payload = { "time": time,
            "pv" : pv,
            "inverter":inverter,
            "wind": wind,
            "grid": grid,
            "load": load}
    headers = {'Content-Type': 'application/json'}
    response = requests.post(url = URL, headers = headers, data = json.dumps(payload));

    #results = response.text
    #print("Requested%s", results);