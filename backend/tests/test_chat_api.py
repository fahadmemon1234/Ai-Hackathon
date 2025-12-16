from fastapi.testclient import TestClient
from fastapi import status
from backend.src.main import app
from unittest.mock import patch, MagicMock
import pytest

client = TestClient(app)

@pytest.fixture
def mock_agent_service():
    with patch("backend.src.api.chat.AgentService") as mock:
        instance = mock.return_value
        instance.get_rag_answer.return_value.response = "Mocked answer from agent."
        instance.get_rag_answer.return_value.session_id = "test-session-id"
        yield instance

def test_chat_full_book_success(mock_agent_service):
    """
    Test the /api/chat endpoint with full_book context mode for a successful response.
    """
    response = client.post(
        "/chat",
        json={
            "message": "What is AI?",
            "context_mode": "full_book"
        }
    )

    assert response.status_code == status.HTTP_200_OK
    data = response.json()
    assert "session_id" in data
    assert data["response"] == "Mocked answer from agent."
    mock_agent_service.get_rag_answer.assert_called_once()

def test_chat_invalid_context_mode():
    """
    Test the /api/chat endpoint with an invalid context mode.
    """
    response = client.post(
        "/chat",
        json={
            "message": "Invalid query",
            "context_mode": "invalid_mode"
        }
    )

    assert response.status_code == status.HTTP_400_BAD_REQUEST
    assert "Invalid context_mode" in response.json()["detail"]

def test_chat_empty_message():
    """
    Test the /api/chat endpoint with an empty message.
    """
    response = client.post(
        "/chat",
        json={
            "message": "",
            "context_mode": "full_book"
        }
    )

    assert response.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY # Pydantic validation error
    assert "field required" in response.json()["detail"][0]["msg"]

def test_chat_selected_text_success(mock_agent_service):
    """
    Test the /api/chat endpoint with selected_text context mode for a successful response.
    """
    response = client.post(
        "/chat",
        json={
            "message": "Explain this concept.",
            "context_mode": "selected_text",
            "selected_text": "This is a selected piece of text about AI."
        }
    )

    assert response.status_code == status.HTTP_200_OK
    data = response.json()
    assert "session_id" in data
    assert data["response"] == "Mocked answer from agent."
    mock_agent_service.get_rag_answer.assert_called_once()
    assert mock_agent_service.get_rag_answer.call_args[0][0].context_mode == "selected_text"
    assert mock_agent_service.get_rag_answer.call_args[0][0].selected_text == "This is a selected piece of text about AI."

def test_chat_selected_text_no_selection():
    """
    Test the /api/chat endpoint with selected_text context mode but no selected_text provided.
    """
    response = client.post(
        "/chat",
        json={
            "message": "Explain this concept.",
            "context_mode": "selected_text"
        }
    )

    assert response.status_code == status.HTTP_400_BAD_REQUEST
    assert "selected_text must be provided" in response.json()["detail"]