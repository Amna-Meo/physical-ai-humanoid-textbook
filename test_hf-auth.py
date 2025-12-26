import os
from dotenv import load_dotenv
from huggingface_hub import HfApi

load_dotenv()

token = os.getenv("HF_TOKEN")

api = HfApi(token=token)
user = api.whoami()

print("Authenticated as:", user["name"])

