using Documenter

try
    using RouteBidModel
catch
    if !("../src/" in LOAD_PATH)
       push!(LOAD_PATH,"../src/")
       @info "Added \"../src/\"to the path: $LOAD_PATH "
       using RouteBidModel
    end
end

makedocs(
    sitename = "RouteBidModel",
    format = format = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true"
    ),
    modules = [RouteBidModel],
    pages = ["index.md", "reference.md"],
    doctest = true
)

deploydocs(
    repo ="github.com/jacfilip/RouteBidModel.jl.git",
    target="build"
)
