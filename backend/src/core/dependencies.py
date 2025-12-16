from qdrant_client import QdrantClient
import cohere
from openai import OpenAI
from backend.src.core.config import settings

def get_qdrant_client() -> QdrantClient:
    return QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY)

def get_cohere_client() -> cohere.Client:
    return cohere.Client(api_key=settings.COHERE_API_KEY)

def get_gemini_client() -> OpenAI: # Assuming python-gemini-sdk uses OpenAI client interface
    # For Gemini, if it uses the OpenAI SDK interface, we can configure it this way.
    # Otherwise, this would need to be the actual Gemini Python SDK client.
    # The plan mentioned "OpenAI Agent SDK" and "custom gemini llm", which implies
    # using the OpenAI client with a custom base_url for Gemini.
    return OpenAI(
        api_key=settings.GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta" # Example base_url for Gemini if using OpenAI client
    )
