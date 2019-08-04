<?php
include "header.inc.php";
?>
<script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>
<script type="text/javascript">
lastrxtime = <?=time()-$range?>;
range = <?=$range?>;
call = '<?=$_GET['call']?>';

function initPage() {
	initMap();
	updateData();
	setInterval(updateData, 5000);
}
</script>
<script type="text/javascript" src="script.js"></script>
<script async defer src="https://maps.googleapis.com/maps/api/js?key=AIzaSyCdZi_9pTemN0IOE2KO_LMkR1-jJh92qhU&callback=initMap"></script>
</head>
<body onload="initPage()">

<?php
include "sidebar.inc.php";
include "topbar.inc.php";
?>

<div id="map" class="inner" style="width:1292px;height:800px;"></div>

</body>
</html>
<?php
include "footer.inc.php";
?>

