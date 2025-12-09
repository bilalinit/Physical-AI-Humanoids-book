import os
from dotenv import load_dotenv
load_dotenv()

import qdrant_client

# Initialize Qdrant client
qdrant_client_instance = qdrant_client.QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Print available methods
print("Available methods in qdrant_client_instance:")
methods = [method for method in dir(qdrant_client_instance) if not method.startswith('_')]
for method in sorted(methods):
    print(f"  {method}")

# Check specifically for search-related methods
search_methods = [method for method in methods if 'search' in method.lower()]
print(f"\nSearch-related methods: {search_methods}")