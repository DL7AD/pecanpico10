<?php
include "header.inc.php";
?>
<script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>
<script type="text/javascript">

lastrxtime = <?=time()-$range?>;
call = '<?=$_GET['call']?>';
var lastChartUpdate = 0;

var map;
var items = [];


function loadRecentData() {
	$.getJSON("ajax/telemetry.php?call=" + call + "&from=" + lastrxtime, function(json) {
		tel = json['telemetry'];

		// Remove old items
		while(true) {
			try {
				items.pop().setMap(null);
			} catch(err) {
				break;
			}
		}

		// Add new items on map
		var last = null;

		// Update charts if there is new data or at a timeout of 300 seconds
		if(tel.length > 0 || json['time']-lastChartUpdate > 300) {

			$.each(tel, function(key, data) {

				if(data.gps_lat || data.gps_lon) {
					// Line between points
					if(last) {
						if(last.gps_lat != last.gps_lon || data.gps_lat != data.gps_lon) {
							var line = new google.maps.Polyline({
								path: [{lng:last.gps_lon/10000000, lat:last.gps_lat/10000000},{lng:data.gps_lon/10000000, lat:data.gps_lat/10000000}],
								geodesic: true,
								strokeColor: last.org == 'log' || data.org == 'log' ? '#FF0000' : '#008000',
								strokeOpacity: 0.4,
								strokeWeight: 5,
								map: map
							});
						}
						items.push(line);
					}
					last = data;
				}
			});


			lastChartUpdate = json['time'];
		}

	});
}
<?php
$tel = (new Tracker($_GET['call']))->getLastTelemetry();
?>
function initMap() {
	map = new google.maps.Map(document.getElementById('map'), {
		zoom: 8,
		center: new google.maps.LatLng(<?=$tel->gps_lat/10000000?>,<?=$tel->gps_lon/10000000?>),
		gestureHandling: 'greedy'
	});

	loadRecentData();
	setInterval(loadRecentData, 1000);
}
</script>
<script async defer src="https://maps.googleapis.com/maps/api/js?key=AIzaSyCdZi_9pTemN0IOE2KO_LMkR1-jJh92qhU&callback=initMap"></script>
</head>
<body>

<?php
include "sidebar.inc.php";
?>

<table style="float:left;">
	<tr>
		<td>
			&nbsp;Range:
			<a href="?call=<?=$_GET['call']?>&range=3600">1h</a>
			<a href="?call=<?=$_GET['call']?>&range=10800">3h</a>
			<a href="?call=<?=$_GET['call']?>&range=21600">6h</a>
			<a href="?call=<?=$_GET['call']?>&range=43200">12h</a>
			<a href="?call=<?=$_GET['call']?>&range=86400">24h</a>
			<a href="?call=<?=$_GET['call']?>&range=172800">2d</a>
			<a href="?call=<?=$_GET['call']?>&range=259200">3d</a>
			<a href="?call=<?=$_GET['call']?>&range=432000">5d</a>
			<a href="?call=<?=$_GET['call']?>&range=604800">7d</a>
			<a href="?call=<?=$_GET['call']?>&range=1209600">14d</a>
			<a href="?call=<?=$_GET['call']?>&range=1814400">21d</a>
			<a href="?call=<?=$_GET['call']?>&range=2592000">30d</a>
		</td>
	</tr>
	<tr>
		<td><div id="map" class="inner" style="width:1292px;height:800px;"></div></td>
	</tr>
</table>

</body>
</html>
<?php
include "footer.inc.php";
?>

