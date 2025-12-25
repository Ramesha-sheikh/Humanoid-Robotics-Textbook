import re

def clean_text(text: str) -> str:
    text = re.sub(r'\n{3,}', '\n\n', text)      # extra newlines remove
    text = re.sub(r' +', ' ', text)             # extra spaces remove
    text = '\n'.join(line.strip() for line in text.split('\n'))
    text = re.sub(r'\s*-\s*', '-', text)
    return text.strip()