# RouteBidModel.jl
Repository accompanying the paper:

*Optimization of the cost of urban traffic through an online bidding platform for commuters*

by Jacek Filipowski, Bogumił Kamiński, Atefeh Mashatan, Paweł Prałat, Przemysław Szufel.


We consider the problem of increasing efficiency of a transportation system through optimizing the behaviour of commuters. The assumption is that the time spent in the traffic can be represented by a monetary value and hence introduction of monetary compensations can lead to a more efficient organization of the transportation system. In our model, heterogeneous travelers differently assess the value of their time spent in congestion, hence it is presumably viable to reduce traffic in the most congested streets by introducing a bidding mechanism that will allow the participants having a lower monetary value of time to receive a compensation financed by the group of commuters having a higher value of time spend in congestion. 

We verifed proposed auction design via an agent based simulation model representing the Manhattan area of New York City. The results of our simulation confirm the theoretical findings that the introduction of the proposed auction mechanism in a real city settings leads to a more efficient allocation of routes chosen by agents.

| **Documentation** | 
|---------------|
|[![][docs-stable-img]][docs-stable-url] <br/> [![][docs-latest-img]][docs-dev-url] |

## Documentation


- [**STABLE**][docs-stable-url] &mdash; **documentation of the most recently tagged version.**
- [**DEV**][docs-dev-url] &mdash; **documentation of the development version.**

[docs-latest-img]: https://img.shields.io/badge/docs-latest-blue.svg
[docs-stable-img]: https://img.shields.io/badge/docs-stable-blue.svg
[docs-dev-url]: https://travis-ci.org/jacfilip/RouteBidModel/dev
[docs-stable-url]: https://travis-ci.org/jacfilip/RouteBidModel/stable

[travis-img]: https://travis-ci.org/jacfilip/RouteBidModel.svg?branch=master
[travis-url]: https://travis-ci.org/jacfilip/RouteBidModel

**Acknowledgements**

This research was funded, in part, through a generous contribution from NXM Labs Inc. NXM’s autonomous security technology enables devices, including connected vehicles, to communicate securely with each other and their surroundings without human intervention while leveraging data at the edge to provide business intelligence and insights. NXM ensures data privacy and integrity by using a novel blockchain-based architecture which enables rapid and regulatory-compliant data monetization.
