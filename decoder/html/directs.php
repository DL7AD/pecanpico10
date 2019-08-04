<?php
include "header.inc.php";
?>
<script type="text/javascript">
lastrxtime = <?=time()-$range?>;
range = <?=$range?>;
call = '<?=$_GET['call']?>';
filter = '<?=$_GET['filter']?>';

function loadDirects() {
	updateData(true);
	setInterval(function(){updateData(true);}, 1000);
}
</script>
<script type="text/javascript" src="script.js"></script>
</head>
<body onload="loadDirects()">

<?php
include "sidebar.inc.php";
include "topbar.inc.php";
?>

<div id="directs"></div>

</body>
</html>
<?php
include "footer.inc.php";
?>

