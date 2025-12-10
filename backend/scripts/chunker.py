import re
from pathlib import Path
import tiktoken
from typing import List, Dict

class DocumentChunker:
    def __init__(self, max_tokens: int = 512):
        self.max_tokens = max_tokens
        self.tokenizer = tiktoken.get_encoding("cl100k_base")

    def count_tokens(self, text: str) -> int:
        return len(self.tokenizer.encode(text))

    def parse_markdown(self, file_path: Path) -> List[Dict]:
        """Parse markdown and extract heading hierarchy"""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        chunks = []
        current_h2 = None
        current_h3 = None
        current_content = []

        for line in content.split('\n'):
            h2_match = re.match(r'^##\s+(.+)$', line)
            h3_match = re.match(r'^###\s+(.+)$', line)

            if h2_match:
                # Flush previous section
                if current_content:
                    chunk = self._create_chunk(
                        file_path, current_h2, current_h3, current_content
                    )
                    if chunk:
                        chunks.append(chunk)
                current_h2 = h2_match.group(1)
                current_h3 = None
                current_content = []
            elif h3_match:
                # Flush previous subsection
                if current_content:
                    chunk = self._create_chunk(
                        file_path, current_h2, current_h3, current_content
                    )
                    if chunk:
                        chunks.append(chunk)
                current_h3 = h3_match.group(1)
                current_content = []
            else:
                if line.strip():
                    current_content.append(line)

        # Flush final section
        if current_content:
            chunk = self._create_chunk(
                file_path, current_h2, current_h3, current_content
            )
            if chunk:
                chunks.append(chunk)

        return chunks

    def _create_chunk(self, file_path: Path, h2: str, h3: str, lines: List[str]) -> Dict:
        """Create chunk with metadata"""
        content = '\n'.join(lines).strip()

        # Skip chunks with empty content
        if not content:
            return None

        heading_path = [h for h in [h2, h3] if h]

        # Split if exceeds max tokens
        token_count = self.count_tokens(content)
        if token_count > self.max_tokens:
            content = self._truncate_to_tokens(content, self.max_tokens)
            token_count = self.max_tokens

        # Find docs directory in path and get relative path from there
        path_parts = file_path.parts
        try:
            docs_index = path_parts.index('docs')
            relative_path = Path(*path_parts[docs_index:])
        except ValueError:
            relative_path = file_path.name

        return {
            "file_path": str(relative_path),
            "heading_path": heading_path,
            "section_depth": len(heading_path),
            "content": content,
            "token_count": token_count
        }

    def _truncate_to_tokens(self, text: str, max_tokens: int) -> str:
        """Truncate text to max tokens at paragraph boundary"""
        paragraphs = text.split('\n\n')
        result = []
        current_tokens = 0

        for para in paragraphs:
            para_tokens = self.count_tokens(para)
            if current_tokens + para_tokens <= max_tokens:
                result.append(para)
                current_tokens += para_tokens
            else:
                break

        return '\n\n'.join(result)
