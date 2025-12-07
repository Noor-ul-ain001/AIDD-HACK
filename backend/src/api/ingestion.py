from fastapi import APIRouter, HTTPException
import os
import hashlib
from backend.src.services.ingestion import ingest_chapter_content, create_qdrant_collection

router = APIRouter()

async def get_all_md_files(root_dir):
    md_files = []
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(".md") or filename.endswith(".mdx"):
                md_files.append(os.path.join(dirpath, filename))
    return md_files

@router.post("/ingest", status_code=201)
async def ingest_docs():
    """
    Ingests all markdown documents from the frontend/docs directory.
    """
    try:
        await create_qdrant_collection()

        docs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "frontend", "docs"))
        md_files = await get_all_md_files(docs_path)

        if not md_files:
            raise HTTPException(status_code=404, detail="No markdown files found in frontend/docs.")

        for file_path in md_files:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()

            title = os.path.basename(file_path) # Default to filename
            for line in content.split("\n"):
                if line.startswith("# "):
                    title = line[2:].strip()
                    break
                elif line.startswith("## "):
                    title = line[3:].strip()
                    break

            file_id = hashlib.sha256(file_path.encode()).hexdigest()

            await ingest_chapter_content(
                chapter_id=file_id,
                chapter_title=title,
                chapter_content=content,
            )

        return {"status": "success", "message": f"Ingested {len(md_files)} documents."}

    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
