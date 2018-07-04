<?php
include "header.inc.php";
?>
<script type="text/javascript">

lastrxtime = <?=time()-$range?>;
range = <?=$range?>;
call = '<?=$_GET['call']?>';

function loadImages() {
	updateData();
	setInterval(updateData, 5000);
}
</script>
<script type="text/javascript" src="script.js"></script>
</head>
<body onload="loadImages()">

<?php
include "sidebar.inc.php";
?>

<table>
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
</table>

<div style="width:1330px;float:left;" id="images"></div>

</body>
</html>
<?php
include "footer.inc.php";
?>

