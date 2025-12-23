from openai import OpenAI
from app.core.config import settings
import logging
from typing import AsyncIterator

logger = logging.getLogger(__name__)


class LLMService:
    def __init__(self):
        # Configure OpenAI client for Gemini
        self.client = OpenAI(
            api_key=settings.gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        self.model = settings.gemini_model

    async def stream_completion(
        self,
        system_prompt: str,
        user_message: str,
        context_chunks: list[str] = None
    ) -> AsyncIterator[str]:
        """Stream Gemini response via OpenAI SDK"""

        # Build context
        context_text = ""
        if context_chunks:
            context_text = "\n\n".join([
                f"[Source {i+1}]: {chunk}"
                for i, chunk in enumerate(context_chunks)
            ])

        messages = [
            {"role": "system", "content": system_prompt},
        ]

        if context_text:
            messages.append({
                "role": "user",
                "content": f"Context:\n{context_text}\n\nQuestion: {user_message}"
            })
        else:
            messages.append({"role": "user", "content": user_message})

        try:
            stream = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                stream=True,
                temperature=0.7,
                max_tokens=1000
            )

            for chunk in stream:
                if chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content

        except Exception as e:
            logger.error(f"LLM streaming error: {e}")
            yield f"Error: {str(e)}"


llm_service = LLMService()
