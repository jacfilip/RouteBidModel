module Visuals

using Compose
using DataFrames

include("./decls.jl")

function MarkPositions(lats::Vector{Real}, lons::Vector{Real})
    pts = [(lats[i], lons[i], 10.) for i in 1:length(lats)]
    compose(context(), circle(lons, lats, [10.]), fill("red"), linewidth(0mm))
end

function DrawGraph(pts::Vector{Tuple{Tuple{Real,Real},Tuple{Real,Real}}})
    maxLat, maxLon = maximum((i -> maximum([i[1][1], i[2][1]])).(pts)),  maximum((i -> maximum([i[1][2], i[2][2]])).(pts))
    minLat, minLon = minimum((i -> minimum([i[1][1], i[2][1]])).(pts)),  minimum((i -> minimum([i[1][2], i[2][2]])).(pts))
    w, h = maxLon - minLon, maxLat - minLat
    compose(context(units = UnitBox(minLon - 0.1 * w, minLat - 0.1 * h, maxLon + 0.2 * w, maxLat + 0.2 * h)), line(pts), linewidth(1mm), stroke("white"))
end

end  #module
