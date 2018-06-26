<div class="inner side">
	<?php
	$lasthour = false;
	$today = false;
	$yesterday = false;
	$older = false;

	$trackers = $db->getTracker();
	usort($trackers, function($a, $b) {
		return $a->getLastActivity()['pkt'] > $b->getLastActivity()['pkt'] ? 1 : -1;
	});
	
	foreach($trackers as $tr) {

		$cnt = $tr->getPacketCount();
		$act = min($tr->getLastActivity());
		if($act <= 86400 and !$last24h) {
			echo "<p><b>Last 24h</b></p>";
			$last24h = true;
		}
		if($act > 86400 and $act <= 86400*7 and !$yesterday) {
			echo "<p><b>Yesterday</b></p>";
			$yesterday = true;
		}
		if($act > 86400*7 and $act <= 86400*30 and !$aweekago) {
			echo "<p><b>A week ago</b></p>";
			$aweekago = true;
		}
		if($act > 86400*30 and !$amonthago) {
			echo "<p><b>A month ago</b></p>";
			$amonthago = true;
		}

		echo "<div class=\"call\">
		<b><a href=\"telemetry.php?call=" . $tr->getCall() . "\">" . $tr->getCall() . "</a> ...
		<a href=\"map.php?call=" . $tr->getCall() . "\">Map</a>
		<a href=\"images.php?call=" . $tr->getCall() . "\">Images</a></b><br>
		Last Activity: " . time_format($act) . "<br>
		Packets: " . number_format($cnt['img']['cnt300'] + $cnt['pos']['cnt300']) . " (5m), " . number_format($cnt['img']['cnt3600'] + $cnt['pos']['cnt3600']) . " (1h)
		</div>";

	}
	?>
</div>
