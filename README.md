# Bicycle-Redistribution-Project
Case Studies, Project 3. Bicycle Redistribution Project


Model and data assumptions:
•	Static variant of the problem, where user activities are neglected during rebalancing.
•	The entire domain is divided into subregions, and one station is assigned to each one of them.
•	Effort for distributing bikes inside a subregion is not considered. 
•	The system of stations and routes are considered as a complete direct graph.
•	Operational costs consist only of travel distances between stations. 
•	The total demand of the stations is equal to the total supply, so the perfect balance could be achieved without changing the given number of bikes in the system.

Vehicle assumptions:
•	All vehicles are homogeneous, i.e. all of them have the same load capacity, which can never be exceeded. 
•	All the vehicles start and finish from the one depot. 
•	Each vehicle is empty at the start of a trip and must be empty at the moment of arrival back to the depot.
The presumptions of homogeneity and depot location being fixed are not crucial and can be easily omitted.

Routing assumptions:
•	Stations may be visited multiple times by the same or different vehicles.
•	Monotonicity regarding pickup and delivery operations. A vehicle is only allowed to load bicycles at load stations and pickup bicycles from pickup stations. In other words, the number of bikes at each station is allowed to only decrease or only increase.


TODO: describe how to use code
