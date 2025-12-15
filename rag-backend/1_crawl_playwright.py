import asyncio
from playwright.async_api import async_playwright
import trafilatura
import json
from urllib.parse import urljoin
import requests
from bs4 import BeautifulSoup

SITEMAP_URL = "https://humanoid-robotics-textbook-psi.vercel.app/sitemap.xml"
OUTPUT_FILE = "rag-backend/data/book_pages_playwright.json"

async def crawl_with_playwright():
    urls = []
    # Get URLs from sitemap
    resp = requests.get(SITEMAP_URL)
    soup = BeautifulSoup(resp.content, "xml")
    for loc in soup.find_all("loc"):
        urls.append(loc.text.strip())

    results = []
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True)
        context = await browser.new_context(
            user_agent="Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36"
        )
        page = await context.new_page()

        for url in urls:
            try:
                print(f"Crawling: {url}")
                await page.goto(url, wait_until="domcontentloaded", timeout=30000)
                await page.wait_for_timeout(2000)  # Let JS load
                html = await page.content()
                text = trafilatura.extract(html)
                title = await page.title()
                if text and len(text.strip()) > 100:
                    results.append({"url": url, "title": title, "text": text.strip()})
                    print(f"Success: {title[:50]}...")
            except Exception as e:
                print(f"Failed {url}: {e}")

        await browser.close()

    # Save
    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    print(f"DONE! {len(results)} pages saved to {OUTPUT_FILE}")

asyncio.run(crawl_with_playwright())