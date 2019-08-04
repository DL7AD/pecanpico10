<?php
require_once "Database.class.php";
require_once "Telemetry.class.php";
require_once "Image.class.php";
require_once "Raw.class.php";
require_once "Directs.class.php";

class Tracker {

	private $call;
	private $lastActivity;

	function __construct($call) {
		$this->call = $call;
	}

	function getLastActivity() {
		if(!is_null($this->lastActivity)) // Read from cache
			return $this->lastActivity;

		$act = array();

		$query = Database::getInstance()->query("
			SELECT * FROM (
				(SELECT UNIX_TIMESTAMP() - `rxtime` as `lasttime`,'pos' as `type` FROM `position` WHERE `call` = '" . Database::getInstance()->escape_string($this->call) . "' AND `org` = 'pos' ORDER BY `rxtime` DESC LIMIT 1)
				UNION ALL
				(SELECT UNIX_TIMESTAMP() - `rxtime` as `lasttime`,'dir' as `type` FROM `directs` WHERE `call` = '" . Database::getInstance()->escape_string($this->call) . "' ORDER BY `rxtime` DESC LIMIT 1)
				UNION ALL
				(SELECT UNIX_TIMESTAMP() - `rxtime` as `lasttime`,'img' as `type` FROM `image` WHERE `call` = '" . Database::getInstance()->escape_string($this->call) . "' ORDER BY `rxtime` DESC LIMIT 1)
				UNION ALL
				(SELECT UNIX_TIMESTAMP() - `rxtime` as `lasttime`,'log' as `type` FROM `position` WHERE `call` = '" . Database::getInstance()->escape_string($this->call) . "' AND `org` = 'log' ORDER BY `rxtime` DESC LIMIT 1)
			) AS d
		");

		while($row = $query->fetch_assoc())
			$act[$row['type']] = $row['lasttime'];

		$act['pkt'] = min($act);

		$this->lastActivity = $act;
		return $this->lastActivity;
	}
	function getPictures($from, $to=NULL) {
		if(is_null($to))
			$to = time() + 1;

		if($from > $to)
			return array(); // Error $from is larger than $to

		if($to - $from > 64281600)
			$from = $to - 64281600; // Max. 744 days (2 non leap years + 14 weeks)

		$query = Database::getInstance()->query("
			SELECT t.`id`,`call`,MIN(`rxtime`) as `time_first`,MAX(`rxtime`) as `time_last`,
			COUNT(*) as `count`,`imageID`,MAX(`packetID`) as `packetID`
			FROM (
				SELECT `id`
				FROM `image`
				WHERE " . intval($from) . " <= `rxtime`
				AND `rxtime` <= " . intval($to) . "
				AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				GROUP BY `id`
				ORDER BY `id` ASC
			) as s
			JOIN image t ON t.`id` = s.`id`
			GROUP BY t.`id`
		");

		$pics = array();
		while($row = $query->fetch_assoc())
			$pics[] = new Image($row);

		return $pics;
	}
	function getLastTelemetry() {
		$query = Database::getInstance()->query("SELECT * FROM `position` WHERE `call` = '" . Database::getInstance()->escape_string($this->call) . "' ORDER BY `rxtime` DESC LIMIT 1");
		return new Telemetry($query->fetch_assoc());
	}
	function getTelemetry($from, $to=NULL) {
		if(is_null($to))
			$to = time() + 1;

		if($from > $to)
			return array(); // Error $from is larger than $to

		if($to - $from > 64281600)
			$from = $to - 64281600; // Max. 744 days (2 non leap years + 14 weeks)

		$query = Database::getInstance()->query("
			SELECT *,
			CASE
				WHEN `org` = 'pos' THEN `rxtime`
				WHEN `org` = 'log' THEN `gps_time`
			END AS `ordertime`,
			MAX(`org`) as `org`
			FROM `position`
			WHERE ((
				" . intval($from) . " <= `rxtime`
				AND `rxtime` <= " . intval($to) . "
				AND `org` = 'pos'
			) OR (
				" . intval($from) . " <= `gps_time`
				AND `gps_time` <= " . intval($to) . "
				AND `org` = 'log'
			))
			AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
			GROUP BY `reset`,`id`
			ORDER BY `ordertime` ASC
		");

		$datasets = array();
		while($row = $query->fetch_assoc()) {
			$datasets[] = new Telemetry($row);
		}

		return $datasets;
	}
	function getRaw($from, $to=NULL) {
		if(is_null($to))
			$to = time() + 1;

		if($from > $to)
			return array(); // Error $from is larger than $to

		if($to - $from > 64281600)
			$from = $to - 64281600; // Max. 744 days (2 non leap years + 14 weeks)

		$query = Database::getInstance()->query("
			SELECT *
			FROM `raw`
			WHERE " . intval($from) . " <= `rxtime`
			AND `rxtime` <= " . intval($to) . "
			AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
			ORDER BY `rxtime` ASC
		");

		$datasets = array();
		while($row = $query->fetch_assoc()) {
			$datasets[] = new Raw($row);
		}

		return $datasets;
	}
	function getPacketCount() {
		$query = Database::getInstance()->query("SELECT *
			FROM (
				SELECT COUNT(`id`) as `cnt86400`,'pos' as `type` FROM `position` WHERE `org` = 'pos' AND `rxtime`+86400 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				UNION ALL
				SELECT COUNT(`rxtime`) as `cnt86400`,'dir' as `type` FROM `directs` WHERE `rxtime`+86400 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				UNION ALL
				SELECT COUNT(`id`) as `cnt86400`,'img' as `type` FROM `image` WHERE `rxtime`+86400 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				UNION ALL
				SELECT COUNT(`id`) as `cnt86400`,'log' as `type` FROM `position` WHERE `org` = 'log' AND `rxtime`+86400 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
			) AS a
			JOIN (
				SELECT COUNT(`id`) as `cnt3600`,'pos' as `type` FROM `position` WHERE `org` = 'pos' AND `rxtime`+3600 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				UNION ALL
				SELECT COUNT(`rxtime`) as `cnt3600`,'dir' as `type` FROM `directs` WHERE `rxtime`+3600 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				UNION ALL
				SELECT COUNT(`id`) as `cnt3600`,'img' as `type` FROM `image` WHERE `rxtime`+3600 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				UNION ALL
				SELECT COUNT(`id`) as `cnt3600`,'log' as `type` FROM `position` WHERE `org` = 'log' AND `rxtime`+3600 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
			) AS b
			JOIN (
				SELECT COUNT(`id`) as `cnt300`,'pos' as `type` FROM `position` WHERE `org` = 'pos' AND `rxtime`+300 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				UNION ALL
				SELECT COUNT(`rxtime`) as `cnt300`,'dir' as `type` FROM `directs` WHERE `rxtime`+300 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				UNION ALL
				SELECT COUNT(`id`) as `cnt300`,'img' as `type` FROM `image` WHERE `rxtime`+300 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
				UNION ALL
				SELECT COUNT(`id`) as `cnt300`,'log' as `type` FROM `position` WHERE `org` = 'log' AND `rxtime`+300 > UNIX_TIMESTAMP() AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
			) AS c
			WHERE a.`type` = b.`type`
			AND a.`type` = c.`type`
		");


		$ret = array();
		while($row = $query->fetch_assoc())
			$ret[$row['type']] = $row;

		return $ret;
	}
	function getCall() {
		return $this->call;
	}
	function getDirects($from, $to=NULL) {
		if(is_null($to))
			$to = time() + 1;

		if($from > $to)
			return array(); // Error $from is larger than $to

		if($to - $from > 64281600)
			$from = $to - 64281600; // Max. 744 days (2 non leap years + 14 weeks)

		$query = Database::getInstance()->query("
			SELECT *
			FROM `directs`
			WHERE " . intval($from) . " <= `rxtime`
			AND `rxtime` <= " . intval($to) . "
			AND `call` = '" . Database::getInstance()->escape_string($this->call) . "'
			ORDER BY `rxtime` ASC
		");

		$datasets = array();
		while($row = $query->fetch_assoc()) {
			$datasets[] = new Directs($row);
		}

		return $datasets;
	}
}
?>

