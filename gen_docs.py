import ast
import os
import re
from collections import defaultdict


def slugify(text):
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")


def extract_docstrings_from_file(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        source = f.read()
    try:
        tree = ast.parse(source)
    except SyntaxError as e:
        print(f"Syntax error in {filepath}: {e}")
        return []
    return extract_classes(tree)


def extract_classes(tree):
    classes = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            class_name = node.name
            class_doc = ast.get_docstring(node)
            methods = []
            for child in node.body:
                if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    method_doc = ast.get_docstring(child)
                    if not method_doc:
                        method_doc = ""
                    methods.append((child.name, method_doc))
            classes.append((class_name, class_doc, methods))
    return classes


def write_class_md(base_output_dir, folder, class_name, class_doc, methods):
    slug = slugify(class_name)
    output_folder = os.path.join(base_output_dir, folder)
    os.makedirs(output_folder, exist_ok=True)

    filepath = os.path.join(output_folder, f"{slug}.md")
    rel_slug = f"reference/{slugify(folder)}/{slug}"

    with open(filepath, "w", encoding="utf-8") as f:
        f.write(f"---\ntitle: {class_name}\nslug: {rel_slug}\n---\n\n")
        f.write(f"# {class_name}\n\n")
        if class_doc:
            f.write(class_doc + "\n\n")
        if methods:
            f.write("## Methods\n\n")
            for name, doc in methods:
                f.write(f"### {name}()\n\n{doc}\n\n")
    return slug, rel_slug


def capitalize_folder(name):
    return name.capitalize()


def js_str(s):
    return s.replace("\\", "\\\\").replace("'", "\\'")


def generate_sidebar_groups(sidebar_groups):
    lines = []
    for folder, items in sorted(sidebar_groups.items()):
        label = capitalize_folder(folder)
        group_lines = []
        for item in items:
            group_lines.append(
                f"{{ label: '{js_str(item['label'])}', slug: '{js_str(item['slug'])}' }}"
            )
        group_body = ",\n    ".join(group_lines)
        group = (
            "{\n"
            f"  label: '{js_str(label)}',\n"
            f"  items: [\n    {group_body}\n  ]\n"
            "}"
        )
        lines.append(group)
    return ",\n".join(lines)


def main(src_dir, output_dir, dir):
    sidebar_groups = defaultdict(list)

    for dirpath, _, filenames in os.walk(src_dir):
        for fname in filenames:
            if fname.endswith(".py"):
                filepath = os.path.join(dirpath, fname)
                rel_dir = os.path.relpath(dirpath, src_dir)
                rel_folder = rel_dir.split(os.sep)[-1] if rel_dir != "." else "root"
                classes = extract_docstrings_from_file(filepath)
                for class_name, class_doc, methods in classes:
                    slug, rel_slug = write_class_md(
                        output_dir, rel_folder, class_name, class_doc, methods
                    )
                    sidebar_groups[rel_folder].append(
                        {"label": class_name, "slug": rel_slug}
                    )

    sidebar_items = generate_sidebar_groups(sidebar_groups)

    astro_config = (
    f"""
        // @ts-check
        import {{ defineConfig }} from 'astro/config';
        import starlight from '@astrojs/starlight';
        import node from '@astrojs/node';

        export default defineConfig({{
        output: 'server',
        adapter: node({{
            mode: 'standalone'
        }}),
        integrations: [
            starlight({{
            title: 'LemonLib documentation',
            favicon: '/lemons.ico',
            social: [
                {{ icon: 'github', label: 'GitHub', href: 'https://github.com/FRC5113/LemonLib' }},
                {{ icon: 'instagram', label: 'Instagram', href: 'https://www.instagram.com/frc5113/' }}
            ],
            sidebar: [
                {{
                label: 'Guides',
                items: [
                    {{ label: 'Install', slug: 'guides/install' }}
                ]
                }},
                {{
                label: 'Reference',
                items: [
        {sidebar_items}
                ]
                }}
            ]
            }})
        ]
        }});
    """)
    astro_dir = os.path.join(dir, "astro.config.mjs")
    with open(astro_dir, "w", encoding="utf-8") as f:
        f.write(astro_config)

    print("Markdown files written to:", output_dir)
    print("astro.config.mjs generated with nested sidebar")


if __name__ == "__main__":
    import sys

    src_directory = os.path.join("lemonlib")
    main_dir = os.path.join("docs")
    output_path = os.path.join(main_dir, "src", "content", "docs", "reference")
    main(src_directory, output_path, main_dir)
