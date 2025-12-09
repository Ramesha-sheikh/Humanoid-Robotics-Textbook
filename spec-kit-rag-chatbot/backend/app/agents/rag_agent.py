import os
import sys
from typing import List
from openai_agents import Agent, OpenAILLM, OpenAIChatModel, OpenAITool
from dotenv import load_dotenv

# Adjust the path to import from parent directories
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from app.agents.tools import retrieve_relevant_chunks

load_dotenv()

GROQ_API_KEY = os.getenv("GROQ_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

if not GROQ_API_KEY and not OPENAI_API_KEY:
    raise ValueError("Either GROQ_API_KEY or OPENAI_API_KEY must be set.")

# Define Groq LLM
groq_llm = OpenAILLM(
    model=OpenAIChatModel(model_name="llama-3.1-70b-instant"), # Primary LLM
    api_key=GROQ_API_KEY
)

# Define OpenAI LLM (fallback)
openai_llm = OpenAILLM(
    model=OpenAIChatModel(model_name="gpt-4o-mini"), # Fallback LLM
    api_key=OPENAI_API_KEY
)

class RAGAgent:
    def __init__(self):
        self.agent = Agent(
            llm=groq_llm,  # Use Groq as primary LLM
            tools=[
                OpenAITool(retrieve_relevant_chunks)
            ],
            system_prompt=(
                "You are an expert AI assistant specializing in humanoid robotics and physical AI, designed to answer questions from a comprehensive textbook. "
                "Your responses must be accurate, concise, and directly derived from the provided document chunks. "
                "When answering, always cite the source title and URL from the chunks if available. "
                "You can answer in a natural mix of English and Roman Urdu. "
                "If the user asks a question, first use the `retrieve_relevant_chunks` tool to find information. "
                "If the `retrieve_relevant_chunks` tool returns no relevant information, state that you cannot find the answer in the provided textbook. "
                "Do not make up answers or provide information not present in the retrieved content. "
                "The user expects a response within 5-8 seconds, so be efficient. "
                "Format your answers clearly, including source links as markdown hyperlinks (e.g., [Title](/docs/chapter/section))."
            )
        )

    async def run_stream(self, user_query: str) -> str:
        """
        Runs the RAG agent to generate a streaming response to a user query.

        Args:
            user_query: The user's question.

        Yields:
            str: Chunks of the agent's response.
        """
        print(f"RAG Agent received query: {user_query}")
        full_response = ""
        try:
            async for chunk in self.agent.run_stream(user_query):
                full_response += chunk
                yield chunk
        except Exception as e:
            print(f"RAG Agent encountered an error: {e}")
            yield f"Sorry, I encountered an error: {e}"

if __name__ == "__main__":
    async def main():
        # Example usage: ensure environment variables are set (COHERE_API_KEY, GROQ_API_KEY/OPENAI_API_KEY)
        # Also ensure Qdrant is running and populated with data using ingest.py

        rag_agent = RAGAgent()

        # Example 1: English query
        print("--- Example 1: English Query ---")
        english_query = "What are the main types of robots?"
        print(f"Query: {english_query}")
        response_chunks = [chunk async for chunk in rag_agent.run_stream(english_query)]
        print(f"Response: {''.join(response_chunks)}\n")

        # Example 2: Roman Urdu query
        print("--- Example 2: Roman Urdu Query ---")
        urdu_query = "ReAct loop kaise kaam karta hai?"
        print(f"Query: {urdu_query}")
        response_chunks = [chunk async for chunk in rag_agent.run_stream(urdu_query)]
        print(f"Response: {''.join(response_chunks)}\n")

    asyncio.run(main())
