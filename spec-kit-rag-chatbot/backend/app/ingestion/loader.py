import os
import re
import yaml
from typing import List, Dict, Any
from markdown import markdown

class DocumentLoader:
    def __init__(self, docs_base_path: str, chunk_size: int = 600, chunk_overlap: int = 100):
        self.docs_base_path = docs_base_path
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def load_documents(self) -> List[Dict[str, Any]]:
        documents = []
        for root, _, files in os.walk(self.docs_base_path):
            for file_name in files:
                if file_name.endswith('.mdx'):
                    file_path = os.path.join(root, file_name)
                    document_data = self._process_mdx_file(file_path)
                    if document_data:
                        documents.append(document_data)
        return documents

    def _process_mdx_file(self, file_path: str) -> Dict[str, Any]:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        frontmatter = {}
        markdown_content = content

        # Extract YAML frontmatter
        match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
        if match:
            frontmatter = yaml.safe_load(match.group(1))
            markdown_content = content[match.end():]

        # Convert MDX to basic markdown for text extraction and chunking
        # This is a simplification; a full MDX parser might be needed for complex cases
        text_content = self._mdx_to_markdown_text(markdown_content)

        title = frontmatter.get('title', os.path.basename(file_path).replace('.mdx', ''))
        # Derive Docusaurus URL (assuming docs_base_path is Docusaurus base docs folder)
        relative_path = os.path.relpath(file_path, self.docs_base_path)
        url_slug = os.path.splitext(relative_path)[0]
        docusaurus_url = f"/docs/{url_slug}"

        document = {
            "document_id": str(hash(file_path)), # Simple hash for unique ID
            "title": title,
            "url": docusaurus_url,
            "content": text_content,
            "file_path": file_path,
            "headings": self._extract_headings(markdown_content)
        }
        return document

    def _mdx_to_markdown_text(self, mdx_content: str) -> str:
        # Basic conversion: remove JSX, convert markdown to plain text
        # This will need to be more robust for real-world MDX files
        mdx_content = re.sub(r'<{.*?>.*?</{.*?>', '', mdx_content, flags=re.DOTALL) # Remove JSX components
        mdx_content = re.sub(r'{.*?}', '', mdx_content) # Remove inline JSX
        return mdx_content # Returning raw markdown for now, chunker will handle

    def _extract_headings(self, markdown_content: str) -> List[str]:
        headings = re.findall(r'^(#+\s.*)$', markdown_content, re.MULTILINE)
        return [h.strip('# ').strip() for h in headings]

    def chunk_document(self, document: Dict[str, Any]) -> List[Dict[str, Any]]:
        text = document['content']
        chunks = []
        current_chunk_text = []
        current_chunk_metadata = {}
        current_length = 0
        chunk_id_counter = 0

        # Split text by lines or semantic units (e.g., paragraphs, headings)
        lines = text.split('\n')

        for line in lines:
            line_length = len(line.split())

            if current_length + line_length <= self.chunk_size:
                current_chunk_text.append(line)
                current_length += line_length
            else:
                if current_chunk_text:
                    chunks.append({
                        "chunk_id": f"{document["document_id"]}-{chunk_id_counter}",
                        "document_id": document["document_id"],
                        "text": "\n".join(current_chunk_text),
                        "metadata": current_chunk_metadata.copy() # Add metadata here (e.g., headings, URL)
                    })
                    chunk_id_counter += 1

                # Start new chunk with overlap
                current_chunk_text = current_chunk_text[-self.chunk_overlap:] if self.chunk_overlap > 0 else []
                current_length = sum(len(l.split()) for l in current_chunk_text) + line_length
                current_chunk_text.append(line)

        if current_chunk_text:
            chunks.append({
                "chunk_id": f"{document["document_id"]}-{chunk_id_counter}",
                "document_id": document["document_id"],
                "text": "\n".join(current_chunk_text),
                "metadata": current_chunk_metadata.copy()
            })

        # Enhance metadata for each chunk
        for chunk in chunks:
            # Add document-level metadata
            chunk["metadata"]["document_title"] = document["title"]
            chunk["metadata"]["document_url"] = document["url"]
            chunk["metadata"]["file_path"] = document["file_path"]

            # Find nearest heading for semantic context
            chunk_start_index = text.find(chunk["text"])
            if chunk_start_index != -1:
                relevant_headings = [h for h in document["headings"] if text.find(h) < chunk_start_index]
                if relevant_headings:
                    chunk["metadata"]["nearest_heading"] = relevant_headings[-1]
                else:
                    chunk["metadata"]["nearest_heading"] = document["title"]
            else:
                chunk["metadata"]["nearest_heading"] = document["title"]

        return chunks


if __name__ == "__main__":
    # Example usage: Assuming 'docs' is in the parent directory of spec-kit-rag-chatbot
    # For testing, you might need to adjust the path or create dummy .mdx files.
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Adjust docs_base_path to point to the actual Docusaurus docs folder
    # e.g., if docs is at the project root, and this script is in spec-kit-rag-chatbot/backend/app/ingestion
    docs_path = os.path.join(current_dir, "..", "..", "..", "..", "docs") # This might need adjustment based on actual project structure

    # Create a dummy docs folder and mdx file for testing
    if not os.path.exists(docs_path):
        os.makedirs(docs_path)

    dummy_mdx_content = """
---
title: Introduction to Robotics
---

# Chapter 1: Basics of Robotics

This is an introduction to robotics. Robots are cool.

## Section 1.1: What is a Robot?

A robot is a machine—especially one programmable by a computer—capable of carrying out a complex series of actions automatically.

```python
print("Hello Robot!")
```

### Subsection 1.1.1: Types of Robots

There are many types of robots, including industrial robots, mobile robots, and humanoids.

# Chapter 2: Advanced Concepts

More advanced topics in robotics.
"""
    dummy_mdx_file_path = os.path.join(docs_path, "intro.mdx")
    with open(dummy_mdx_file_path, "w", encoding="utf-8") as f:
        f.write(dummy_mdx_content)

    loader = DocumentLoader(docs_base_path=docs_path, chunk_size=100, chunk_overlap=20)
    documents = loader.load_documents()

    for doc in documents:
        print(f"\nDocument Title: {doc['title']}")
        print(f"Document URL: {doc['url']}")
        print(f"File Path: {doc['file_path']}")
        print(f"Headings: {doc['headings']}")
        # print(f"Content: {doc['content'][:200]}...")

        chunks = loader.chunk_document(doc)
        for i, chunk in enumerate(chunks):
            print(f"  Chunk {i+1} (ID: {chunk['chunk_id']}):")
            print(f"    Text: {chunk['text']}")
            print(f"    Metadata: {chunk['metadata']}")

    # Clean up dummy files
    os.remove(dummy_mdx_file_path)
    os.rmdir(docs_path) # Only if docs_path was created by this script and is empty

