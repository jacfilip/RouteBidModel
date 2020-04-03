using Pkg
Pkg.activate(".")
using RouteBidModel
using DataFrames
using Random
using Distributions
using CSV


Random.seed!(0);

#simulation of bidding behavior
p2 = BidModelParams(N_MAX=200, N=[150,100], ct=rand(LogNormal(3,1),200))
bid, log = play_nash(p2, p2.ct, 600)

CSV.write(raw".\results\bids.csv", log)

bids = deepcopy(log[log.i .==251,:].bid)
bids[199]=300
s = solve_optimal_payment(p2,bids).travel_plan

d = DataFrame(x=s.x, bids=bids, ct = p2.ct)
println(solve_optimal_payment(p2,bids).payments)


using RCall
R"""
library(readxl)
library(readr)
library(dplyr)
library(ggplot2)
bids<- read.csv("./results/bids.csv")
bids <- bids %>% group_by(i) %>% summarize(avg = mean(bid), dev = sd(bid))
bids <- bids %>% mutate(min_s = avg - dev /2, max_s = avg + dev /2)
ggplot(bids) + geom_ribbon(aes(x = i, ymin = min_s, ymax = max_s), fill="grey80") + geom_line(aes(x=i, y=avg), size = 1, color="red3") +
  ylim(0,35) +
  theme_light() +xlab("Step") +
  ylab("Bid ($)") + theme(axis.text.x = element_text(size = 15), axis.text.y = element_text(size = 15), axis.title.x = element_text(size = 20), axis.title.y = element_text(size = 20))
"""
