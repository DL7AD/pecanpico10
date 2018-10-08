<?php
include "header.inc.php";
?>
<script type="text/javascript">

lastrxtime = <?=time()-$range?>;
range = <?=$range?>;
call = '<?=$_GET['call']?>';
filter = '<?=$_GET['filter']?>';

function loadImages() {
	updateData(true);
	setInterval(function(){updateData(true);}, 1000);
}
</script>
<script type="text/javascript" src="script.js"></script>
</head>
<body onload="loadImages()">

<?php
include "sidebar.inc.php";
include "topbar.inc.php";
?>

<div style="width:1330px;float:left;" id="raw"></div>

</body>
</html>
<?php
include "footer.inc.php";
?>

