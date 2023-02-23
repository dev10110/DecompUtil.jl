using DecompUtil
using Documenter

DocMeta.setdocmeta!(DecompUtil, :DocTestSetup, :(using DecompUtil); recursive=true)

makedocs(;
    modules=[DecompUtil],
    authors="Devansh Ramgopal Agrawal <devansh@umich.edu> and contributors",
    repo="https://github.com/dev10110/DecompUtil.jl/blob/{commit}{path}#{line}",
    sitename="DecompUtil.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://dev10110.github.io/DecompUtil.jl",
        edit_link="main",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/dev10110/DecompUtil.jl",
    devbranch="main",
)
