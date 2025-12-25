import requests
import json

# Test translation endpoint
url = "http://localhost:8001/translate"
data = {
    "page_url": "http://localhost:3000/docs/introduction",
    "target_language": "urdu"
}

print("Testing translation endpoint...")
print(f"Request: {json.dumps(data, indent=2)}")

try:
    response = requests.post(url, json=data, timeout=30)
    print(f"\nStatus Code: {response.status_code}")

    result = response.json()
    print(f"\nResponse keys: {list(result.keys())}")
    print(f"Success: {result.get('success')}")

    if result.get('success'):
        translation = result.get('translation', '')
        print(f"\nTranslation length: {len(translation)} chars")
        print(f"Translation preview (first 200 chars):\n{translation[:200]}...")
    else:
        print(f"Error: {result.get('error')}")

except Exception as e:
    print(f"Exception: {e}")
