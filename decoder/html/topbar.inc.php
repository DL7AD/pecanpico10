<div class="inner bigright">
	<table>
		<tr>
			<td>Range:</td>
			<td>
				<?php
				$ranges = array(600,1800,3600,10800,21600,43200,86400,259200,604800,2592000);
				foreach($ranges as $r)
					echo "<a href=\"?call=$_GET[call]&range=" . $r . (isset($_GET['filter']) ? "&filter=" . $_GET['filter'] : "") . "\">" . ($r < 3600 ? ($r/60)."m" : ($r <= 86400 ? ($r/3600)."h" : ($r/86400)."d")) . "</a> ";
				?>
			</td>
		</tr>
		<?php
		if(explode(".php", $_SERVER['REQUEST_URI'])[0] == "/raw") {
		?>
		<tr>
			<td>Packet Type Filter:</td>
			<td>
				<?php
				$filters = array("pos", "dir", "img", "log");
				foreach($filters as $f)
					echo "<a href=\"?call=$_GET[call]&range=$_GET[range]&filter=$f\">" . strtoupper($f) . "</a> ";
				echo "<a href=\"?call=$_GET[call]&range=$_GET[range]\">none</a> ";
				?>
			</td>
		</tr>
		<?php
		}
		?>
	</table>
</div>
