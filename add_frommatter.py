import os

output_dir = "docs-output"

for filename in os.listdir(output_dir):
    if not filename.endswith(".md"):
        continue

    filepath = os.path.join(output_dir, filename)
    with open(filepath, "r+", encoding="utf-8") as f:
        content = f.read()
        f.seek(0)
        title = filename.replace(".md", "").replace("_", " ").title()
        f.write(f"---\ntitle: {title}\n---\n\n{content}")
        f.truncate()
