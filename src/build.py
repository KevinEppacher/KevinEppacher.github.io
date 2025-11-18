import re
import mistune

configStatic = {
    "src": [
        "./mainpage.md",
        "./portfolio.md"
    ],

    "highlight": [
        None,
        "./portfolio.html"
    ],

    "outfile": [
        "../index.html",
        "../portfolio.html"
    ]
}

with open("./mainStructure.html", "r") as file:
    page_code = file.read()

start_match = re.search("<!-- START GENERATED HTML HERE -->", page_code)
end_match = re.search("<!-- END GENERATED HTML HERE -->", page_code)

def buildStaticFile(page_code, start_match, end_match, src, highlight, outfile, content=None):
    if content is None:
        with open(src, "r") as file:
            content = file.read()

    content = mistune.html(content)

    page = "".join([
        page_code[:start_match.end()],
        content,
        page_code[end_match.start():]
    ])

    # if highlight:
    #     pattern = f'<a href="{highlight}">'
    #     match = re.search(pattern, page)
    #     if match:
    #         page = page[:match.end()] + "button-highlight " + page[match.end():]

    with open(outfile, "w") as f:
        f.write(page)

for s, h, o in zip(configStatic["src"], configStatic["highlight"], configStatic["outfile"]):
    buildStaticFile(page_code, start_match, end_match, s, h, o)
