


<!--
The following template is based on:
Best-README-Template
Search for this, and you will find!
>
<!-- PROJECT LOGO -->
<br />
<p align="center">
  <!-- <a href="https://github.com/ivsg-psu/Publications_Conferences_2023CCTA_FeasibleVehicleTrajectories">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a> -->

  <h2 align="center"> Synthesizing Feasible Vehicle Trajectories from Microscopic Traffic Simulations
  </h2>

<p align="center"><img src=".\Images\Friction_Analysis_Map_No_LC.jpg" alt="Friction Utilization Map" width="800" height="500">

  <p align="center">
    This code is capable of providing physically realistic chassis data for individual vehicles operating in larger road networks with network-scale interactions between vehicles, as explained in the manuscript "Synthesizing Feasible Vehicle Trajectories from Microscopic Traffic Simulations".
    <br />
    <a href="https://github.com/ivsg-psu/Publications_Conferences_2023CCTA_FeasibleVehicleTrajectories"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/ivsg-psu/Publications_Conferences_2023CCTA_FeasibleVehicleTrajectories/tree/main/Documents">View Demo</a>
    ·
    <a href="https://github.com/ivsg-psu/Publications_Conferences_2023CCTA_FeasibleVehicleTrajectories/issues">Report Bug</a>
    ·
    <a href="https://github.com/ivsg-psu/Publications_Conferences_2023CCTA_FeasibleVehicleTrajectories/issues">Request Feature</a>
  </p>
</p>



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-project">About the Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="structure">Repo Structure</a>
	    <ul>
	    <li><a href="#directories">Top-Level Directories</li>
	    </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
	    <ul>
	    <li><a href="#Generate-plots-of-a-vehicle-ID-on-a-specific-section-ID">Generate plots of a vehicle ID on a specific section ID</li>
	    </ul>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project

Simulating a vehicle’s motion within a road network has many applications, including autonomous vehicle development and transportation planning. These simulations are expected to include not only predictions of an ego vehicle’s behavior, but also interactions with other vehicles, changing rules-of-road, and environmental factors. Some of these interactions are best predicted by vehicle chassis models and simulations, such as the friction utilization of a vehicle on a roadway in changing weather. However, other interactions, such as between vehicles and the rules-of-road or between the ego vehicle and other large groups of vehicles, are best predicted by traffic simulations. As well, the management of data between traffic and chassis simulations can be difficult due to the large number of vehicles and geographic scale in which they operate. There is a need for methods to integrate these toolsets with each other to where the synergistic function of the tools achieves research needs. This paper identifies a particular gap in functionality between traffic simulations and chassis simulations: that traffic simulations predict infeasible vehicle trajectories from the standpoint of vehicle chassis behavior. This study proposes solutions, and the adapted hybrid simulation method is shown to be capable of providing physically realistic chassis data for individual vehicles operating in larger road networks with network-scale interactions between vehicles.


<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Installation

1.  Make sure to run MATLAB 2020b or higher

2. Clone the repo
   ```sh
   git clone https://github.com/ivsg-psu/Publications_Conferences_2023CCTA_FeasibleVehicleTrajectories
   ```
3. Download the datafiles from <a href="https://pennstateoffice365.sharepoint.com/sites/IntelligentVehiclesandSystemsGroup-Active/Shared%20Documents/Forms/AllItems.aspx?ga=1&id=%2Fsites%2FIntelligentVehiclesandSystemsGroup%2DActive%2FShared%20Documents%2FIVSG%2FGitHubMirror%2FPublications%2FConferences%2F2023%2FPublications%5FConferences%5F2023CCTA%5FRoadNetworkFrictionAnalysis&viewid=aa025233%2D06cc%2D49ea%2Dbed2%2Db847e0f89798"><strong>psu-ivsg data cetner</strong></a> into the /Data folder. 

<!-- STRUCTURE OF THE REPO -->
### Directories
The following are the top level directories within the repository:
<ul>
	<li>/Documents folder: Paper manuscript and code description.</li>
	<li>/Path Lib folder: Path following library with functions used by the code repository.</li>
	<li>/UTM Lib folder: UTM library with functions used by the code repository.</li>
	<li>/VD Lib folder: Vehicle dynamics library with functions used by the code repository.</li>
	<li>/Images folder: Images generated by the code.</li>
</ul>


<!-- USAGE EXAMPLES -->
## Usage
<!-- Use this space to show useful examples of how a project can be used.
Additional screenshots, code examples and demos work well in this space. You may
also link to more resources. -->

### Generate plots of a vehicle ID on a specific section ID
1. Download the datafiles from <a href=""><strong>psu-ivsg data center</strong></a> into the /Data folder. 

2. In the main script, set the variables 'section_ID' and 'vehicle_ID'
   ```sh
   script_createRealisticTrajectories_SCE_3DoFPPC.m
   ```
   Reference trajectory data has been provided for 8 section IDs:
   Section 3905:
   - Set vehicle_id = 2 to analyze a lane change trajectory
   - Set vehicle_id = 3 to analyze a no lane change trajectory
   Section 8242:
   - Set vehicle_id = 2 to analyze a lane change trajectory
   - Set vehicle_id = 3 to analyze a no lane change trajectory
   Section 8859: Arterial driving, Route 322 Offramp
   - Set vehicle_id = 2 to analyze a lane change trajectory
   - Set vehicle_id = 3 to analyze a no lane change trajectory
   Section 8860:
   - Set vehicle_id = 2 to analyze a lane change trajectory
   - Set vehicle_id = 3 to analyze a no lane change trajectory
   Section 8938: Highway driving, Interstate 99
   - Set vehicle_id = 2 to analyze a lane change trajectory
   - Set vehicle_id = 3 to analyze a no lane change trajectory
   Section 14479:
   - Set vehicle_id = 2 to analyze a lane change trajectory
   - Set vehicle_id = 3 to analyze a no lane change trajectory
   Section 19343: Urban driving, traffic light intersection in downtown State College
   - Set vehicle_id = 2 to analyze a lane change trajectory
   - Set vehicle_id = 3 to analyze a no lane change trajectory
   Section 37162: Urban driving, traffic light intersection with 4 lanes of traffic
   - Set vehicle_id = 2 to analyze a lane change trajectory
   - Set vehicle_id = 3 to analyze a no lane change trajectory

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.


## Major release versions
This code is still in development (alpha testing)


<!-- CONTACT -->
## Contact
Sean Brennan - sbrennan@psu.edu

Project Link: [https://github.com/ivsg-psu/Publications_Conferences_2023CCTA_FeasibleVehicleTrajectories](https://github.com/ivsg-psu/Publications_Conferences_2023CCTA_FeasibleVehicleTrajectories)



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation.svg?style=for-the-badge
[contributors-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation.svg?style=for-the-badge
[forks-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/network/members
[stars-shield]: https://img.shields.io/github/stars/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation.svg?style=for-the-badge
[stars-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/stargazers
[issues-shield]: https://img.shields.io/github/issues/ivsg-psu/reFeatureExtraction_Association_PointToPointAssociationpo.svg?style=for-the-badge
[issues-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/issues
[license-shield]: https://img.shields.io/github/license/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation.svg?style=for-the-badge
[license-url]: https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/blob/master/LICENSE.txt




