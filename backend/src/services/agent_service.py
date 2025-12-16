import logging
import asyncio
from typing import Optional, List

from agents import (
    Agent,
    RunConfig,
    AsyncOpenAI,
    OpenAIChatCompletionsModel,
    Runner,
    function_tool,
)

from src.services.qdrant_service import qdrant_service
from src.core.config import settings
import cohere

logger = logging.getLogger(__name__)

# =========================================================
# Embeddings Client (Cohere)
# =========================================================
cohere_client = cohere.Client(settings.cohere_api_key)

# =========================================================
# Gemini Client (via OpenAI-compatible interface)
# =========================================================
external_client = AsyncOpenAI(
    api_key=settings.gemini_api_key,
    base_url=settings.gemini_base_url,
)

# =========================================================
# Chat Model
# =========================================================
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=external_client,
)

run_config = RunConfig(
    model=model,
    model_provider=external_client,
    tracing_disabled=True,
)

# =========================================================
# Qdrant Retrieval Tool
# =========================================================
@function_tool
async def retrieve_from_qdrant(query: str) -> str:
    """
    Retrieves relevant passages strictly from the book stored in Qdrant.
    """
    try:
        embed_response = cohere_client.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query",
        )

        query_vector = embed_response.embeddings[0]

        def _query_qdrant():
            return qdrant_service.client.query_points(
                collection_name="ai-robotics-book",
                query=query_vector,
                limit=5,
                with_payload=True,
            )

        results = await asyncio.to_thread(_query_qdrant)

        if not results or not getattr(results, "points", None):
            return "NO_CONTEXT_FOUND"

        texts: List[str] = []

        for point in results.points:
            score = getattr(point, "score", 0)
            payload = point.payload or {}

            # Ignore very weak matches
            if score < 0.25:
                continue

            text = payload.get("text")
            if text:
                texts.append(text)

        return "\n\n".join(texts) if texts else "NO_CONTEXT_FOUND"

    except Exception as e:
        logger.error("Qdrant retrieval failed", exc_info=True)
        return "RETRIEVAL_ERROR"

# =========================================================
# Agent Definition
# =========================================================
book_agent = Agent(
    name="Humanoid Robotics Assistant",
    instructions="""
You are Humanoid Assistant for the book "Physical AI & Humanoid Robotics".

Rules you must follow strictly:

1. You are ONLY allowed to answer using information retrieved from the book.
2. For every non-greeting user message, you MUST call the retrieval tool.
3. If the retrieved context is empty, weak, or missing, say clearly:
   "I cannot find this information in the book."
4. Do NOT use outside knowledge.
5. Do NOT guess, infer, or expand beyond the provided text.

Greeting behavior:
- If the user greets (hello, hi, hey):
  Respond briefly and invite them to ask about the book.
""",
    tools=[retrieve_from_qdrant],
)

# =========================================================
# Agent Service
# =========================================================
class AgentService:
    def __init__(self):
        self.agent = book_agent
        self.run_config = run_config

    async def get_response(
        self,
        user_message: str,
        context_text: Optional[str] = None,
    ) -> str:
        message = user_message.strip()

        if not message:
            return "Hello, I’m the Humanoid Assistant. Feel free to ask any question from the book."

        result = await Runner.run(
            starting_agent=self.agent,
            input=message,
            run_config=self.run_config,
        )

        return result.final_output

# =========================================================
# Service Instance
# =========================================================
agent_service = AgentService()
