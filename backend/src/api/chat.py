import logging
from fastapi import APIRouter, HTTPException
from src.services.agent_service import agent_service
from src.models.schemas import BookContentChunk, UserQuestion, ChatbotAnswer

logger = logging.getLogger(__name__)
router = APIRouter()

@router.post(
    "/chat",
    response_model=ChatbotAnswer,
    summary="Chat with the AI Book Assistant",
)
async def chat_endpoint(request: UserQuestion):
    try:
        logger.info(f"Received chat request: {request.model_dump()}")

        # Decide context
        context_text = (
            request.selected_text
            if request.context_mode == "selected_text"
            else None
        )

        # Get response from agent
        response_content = await agent_service.get_response(
            user_message=request.message,
            context_text=context_text,
        )

        return ChatbotAnswer(
            session_id=None,
            response=response_content,
            history=[]
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")
