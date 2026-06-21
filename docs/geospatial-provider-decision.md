# Geospatial Provider Decision

Status: accepted on 2026-06-21

MapEverything should only bake provider defaults that are practical for a ROS2 recording workflow. The app fetches data, publishes it as ROS messages, and expects another machine to record the bag, so provider terms must allow transient local caching and downstream recording with attribution metadata.

## Decision

- Built-in satellite imagery default: NASA GIBS WMTS, using MODIS or VIIRS true-color layers. It is global, no-login, stable, and aligned with NASA Earthdata's open-data posture. It is lower resolution than commercial basemaps, but it is the best default for a robotics data product that may be cached and recorded.
- Built-in DEM convenience fallback: Mapzen Terrain Tiles on AWS Open Data, using Terrarium PNG tiles. It is global and no-login, and the current app can fetch it as simple Web Mercator tiles. Because it is a multi-source elevation composite, every published tile must carry Mapzen and source-data attribution metadata.
- Preferred authoritative US DEM provider to add next: USGS 3DEP through The National Map or an official USGS elevation service. Use it ahead of Mapzen when the device is inside US coverage. Keep Mapzen as the global fallback until a similarly simple global open DEM provider is integrated.
- Optional login/API-key providers: Copernicus Data Space, OpenTopography, USGS EarthExplorer/EROS, NASA Earthdata DAAC downloads, Google Maps, Mapbox, Esri, MapTiler, Azure/Bing, Sentinel Hub, and commercial satellite providers. These should be user-configured and disabled for ROS bag recording unless the user has rights that allow caching, recording, and redistribution.

## Satellite Imagery Options

| Provider | Login | Cache and recording fit | Recommendation |
| --- | --- | --- | --- |
| NASA GIBS WMTS | No for public WMTS tiles | Good fit for transient cache and ROS bag recording when attribution, layer, date, CRS, zoom, and tile coordinates are published. | Bake as default imagery provider. |
| USGS Landsat or NAIP through EROS/EarthExplorer | Yes for full EROS download workflows | Good public-data fit after product metadata and attribution are preserved, but access flow is heavier than WMTS. | Add as optional US imagery provider, user-login where required. |
| Copernicus Sentinel imagery through Copernicus Data Space | Yes | Sentinel data is open, but CDSE API downloads require access tokens and registration. | Add as optional global user-login provider. |
| Google Maps satellite | API key/account | Poor fit for ROS bag redistribution because Google Maps content terms restrict storing, exporting, resharing, or rehosting outside the service. | Do not bake; display-only with user key if ever supported. |
| Mapbox, Esri, MapTiler, Azure/Bing | API key/account | Contract/product-specific. Treat as display or user-contract providers, not recordable defaults. | Do not bake keys or enable recorded redistribution by default. |
| Planet, Maxar, BlackSky, SkyFi, other commercial imagery | Account/contract | Rights are purchase-contract-specific. | Enterprise/user-supplied plugin only. |

## DEM Options

| Provider | Login | Cache and recording fit | Recommendation |
| --- | --- | --- | --- |
| USGS 3DEP / The National Map | No for TNM access endpoints checked; some alternate EROS flows require login | Best authoritative US DEM choice. Federal source, but retain product metadata and USGS credit because not every federal-hosted item has identical rights. | Implement next and prefer inside US coverage. |
| Mapzen Terrain Tiles on AWS Open Data | No AWS account required | Good engineering fit as global tiled DEM fallback. Attribution is required and varies by underlying regional source. | Keep baked as convenience fallback, with explicit attribution metadata. |
| OpenTopography global DEM and USGS DEM APIs | API key required | Strong optional DEM aggregator, but not anonymous. | User API-key provider. |
| Copernicus DEM through Copernicus Data Space | Login/token required | Good global DEM option; access requires CDSE credentials. | User-login provider. |
| Mapbox Terrain-RGB, Google Elevation, Esri elevation services, commercial terrain APIs | API key/account | Useful for visualization or user-contract workflows, but not safe as recordable baked defaults. | User-configured only; require explicit recording policy. |

## What Can Be Baked Into The App

- Provider definitions for NASA GIBS and Mapzen Terrain Tiles.
- A provider registry with endpoint templates, tile schemes, CRS, encoding, zoom defaults, attribution text, license/source URL, credential requirement, cache policy, and whether the source is recordable by default.
- Source-policy metadata in each provider definition, including recordable-by-default, transient-cache-only, attribution URL, credential requirement, and whether credentials are required.
- Attribution, license, and source-policy metadata in every `GeoTileInfo` and `GeoRasterTile` message.
- Conservative default zoom/radius/fetch cadence settings.
- Tiny test fixtures or sample tiles for automated tests only.

The app should not bake API keys, user credentials, commercial map tiles, large offline imagery/DEM datasets, or any source whose terms do not permit recorded redistribution.

## Login-Gated Sources

- USGS EarthExplorer/EROS services require registration and login credentials for full download features.
- Copernicus Data Space API downloads require a user access token.
- OpenTopography DEM APIs require an API key.
- NASA Earthdata DAAC downloads commonly require Earthdata Login, even though public GIBS WMTS imagery can be fetched without app credentials.
- Google Maps, Mapbox, Esri, MapTiler, Azure/Bing, Sentinel Hub, and commercial satellite providers require an account, API key, contract, or all three.

## Sources Checked

- NASA Earthdata data and information guidance: https://www.earthdata.nasa.gov/engage/open-data-services-software-policies/data-information-guidance
- NASA GIBS WMTS capabilities: https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/wmts.cgi?SERVICE=WMTS&REQUEST=GetCapabilities
- USGS TNM Access API docs: https://tnmaccess.nationalmap.gov/api/v1/docs
- USA.gov federal government copyright guidance: https://www.usa.gov/government-copyright
- USGS EROS registration: https://ers.cr.usgs.gov/register/
- AWS Open Data Terrain Tiles listing: https://registry.opendata.aws/terrain-tiles/
- Mapzen Terrain Tiles attribution: https://github.com/tilezen/joerd/blob/master/docs/attribution.md
- Copernicus Data Space token docs: https://documentation.dataspace.copernicus.eu/APIs/Token.html
- Copernicus Data Space terms: https://dataspace.copernicus.eu/terms-and-conditions
- OpenTopography API docs: https://portal.opentopography.org/apidocs/
- Google Maps Platform terms: https://cloud.google.com/maps-platform/terms
- Mapbox product terms: https://www.mapbox.com/legal/product-terms
- Esri legal terms: https://www.esri.com/en-us/legal/terms/full-master-agreement
