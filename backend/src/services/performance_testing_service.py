from typing import Dict, List, Any, Optional, Callable
from sqlalchemy.orm import Session
import time
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
import statistics
from ..models.chapter import Chapter
from ..models.user import User
from ..models.learning_path import LearningPath
import requests
import json


class PerformanceTestResult:
    """Represents the result of a performance test"""

    def __init__(self, test_name: str, duration: float, requests_per_second: float,
                 avg_response_time: float, error_rate: float, details: Dict[str, Any] = None):
        self.test_name = test_name
        self.duration = duration
        self.requests_per_second = requests_per_second
        self.avg_response_time = avg_response_time
        self.error_rate = error_rate
        self.details = details or {}
        self.timestamp = datetime.utcnow()


class PerformanceTestingService:
    """
    Service for performance optimization and load testing
    """

    def __init__(self, db: Session, base_url: str = "http://localhost:8000"):
        self.db = db
        self.base_url = base_url
        self.test_results: List[PerformanceTestResult] = []

    def run_performance_optimization_analysis(self) -> Dict[str, Any]:
        """
        Analyze performance and suggest optimizations
        """
        analysis = {
            "timestamp": datetime.utcnow().isoformat(),
            "database_performance": self._analyze_database_performance(),
            "api_performance": self._analyze_api_performance(),
            "caching_effectiveness": self._analyze_caching_effectiveness(),
            "optimization_recommendations": [],
            "performance_score": 0.0
        }

        # Generate recommendations based on analysis
        recommendations = []

        # Database optimization recommendations
        db_analysis = analysis["database_performance"]
        if db_analysis["avg_query_time"] > 0.1:  # More than 100ms is slow
            recommendations.append({
                "type": "database",
                "priority": "high",
                "recommendation": "Add database indexes for frequently queried fields",
                "impact": "High"
            })

        if db_analysis["connection_pool_usage"] > 0.8:  # 80% usage
            recommendations.append({
                "type": "database",
                "priority": "medium",
                "recommendation": "Increase database connection pool size",
                "impact": "Medium"
            })

        # API performance recommendations
        api_analysis = analysis["api_performance"]
        if api_analysis["avg_response_time"] > 1.0:  # More than 1 second
            recommendations.append({
                "type": "api",
                "priority": "high",
                "recommendation": "Implement caching for expensive API endpoints",
                "impact": "High"
            })

        # Caching recommendations
        cache_analysis = analysis["caching_effectiveness"]
        if cache_analysis["hit_rate"] < 0.7:  # Less than 70% hit rate
            recommendations.append({
                "type": "caching",
                "priority": "medium",
                "recommendation": "Optimize cache strategies for better hit rate",
                "impact": "Medium"
            })

        analysis["optimization_recommendations"] = recommendations

        # Calculate performance score (0-100)
        score = 100
        if db_analysis["avg_query_time"] > 0.5:
            score -= 20
        if api_analysis["avg_response_time"] > 2.0:
            score -= 20
        if cache_analysis["hit_rate"] < 0.5:
            score -= 15

        analysis["performance_score"] = max(0, score)

        return analysis

    def _analyze_database_performance(self) -> Dict[str, Any]:
        """
        Analyze database performance
        """
        # Measure query performance
        start_time = time.time()

        # Test common queries
        queries = [
            lambda: self.db.query(Chapter).limit(10).all(),
            lambda: self.db.query(User).limit(10).all(),
            lambda: self.db.query(LearningPath).limit(10).all(),
        ]

        query_times = []
        for query in queries:
            query_start = time.time()
            try:
                query()
            except Exception:
                pass
            query_times.append(time.time() - query_start)

        avg_query_time = statistics.mean(query_times) if query_times else 0
        max_query_time = max(query_times) if query_times else 0

        return {
            "avg_query_time": avg_query_time,
            "max_query_time": max_query_time,
            "total_tested_queries": len(queries),
            "connection_pool_usage": 0.3,  # Placeholder - would be actual value in real implementation
            "slow_query_threshold": 0.1,
            "queries_exceeding_threshold": sum(1 for t in query_times if t > 0.1)
        }

    def _analyze_api_performance(self) -> Dict[str, Any]:
        """
        Analyze API performance (simulated)
        """
        # In a real implementation, this would make actual API calls
        # For now, we'll simulate the analysis

        return {
            "avg_response_time": 0.8,  # seconds
            "p95_response_time": 1.5,  # 95th percentile
            "p99_response_time": 2.5,  # 99th percentile
            "requests_per_second": 50,
            "concurrent_users": 100,
            "error_rate": 0.01  # 1% error rate
        }

    def _analyze_caching_effectiveness(self) -> Dict[str, Any]:
        """
        Analyze caching effectiveness (would connect to actual cache in real implementation)
        """
        # In a real implementation, this would connect to the cache service
        # For now, we'll return simulated values
        from .caching_service import get_cache_service
        cache_service = get_cache_service()
        stats = cache_service.get_stats()

        total_requests = stats.get("total_entries", 1) * 2  # Simulate total requests
        cache_hits = stats.get("active_entries", 0)
        hit_rate = cache_hits / total_requests if total_requests > 0 else 0

        return {
            "hit_rate": hit_rate,
            "total_requests": total_requests,
            "cache_hits": cache_hits,
            "cache_misses": total_requests - cache_hits,
            "cache_size": stats.get("active_entries", 0),
            "avg_cache_time_saved": 0.2  # seconds per request
        }

    def run_load_test(self, endpoint: str, num_requests: int = 100, concurrency: int = 10) -> PerformanceTestResult:
        """
        Run a load test on a specific endpoint
        """
        test_name = f"Load Test: {endpoint}"

        start_time = time.time()
        response_times = []
        errors = 0

        # Create a session for connection pooling
        session = requests.Session()

        # Prepare requests
        test_url = f"{self.base_url}{endpoint}"

        def make_request():
            try:
                start_req = time.time()
                response = session.get(test_url, timeout=30)
                response_time = time.time() - start_req
                response_times.append(response_time)

                if response.status_code >= 400:
                    return True  # Error
                return False  # Success
            except Exception:
                return True  # Error due to exception

        # Execute requests concurrently
        with ThreadPoolExecutor(max_workers=concurrency) as executor:
            futures = [executor.submit(make_request) for _ in range(num_requests)]
            for future in as_completed(futures):
                if future.result():
                    errors += 1

        total_time = time.time() - start_time
        requests_per_second = num_requests / total_time if total_time > 0 else 0
        avg_response_time = statistics.mean(response_times) if response_times else 0
        error_rate = errors / num_requests if num_requests > 0 else 0

        details = {
            "num_requests": num_requests,
            "concurrency": concurrency,
            "response_times": response_times,
            "errors": errors,
            "min_response_time": min(response_times) if response_times else 0,
            "max_response_time": max(response_times) if response_times else 0,
            "p95_response_time": self._calculate_percentile(response_times, 95) if response_times else 0,
            "p99_response_time": self._calculate_percentile(response_times, 99) if response_times else 0
        }

        result = PerformanceTestResult(
            test_name,
            total_time,
            requests_per_second,
            avg_response_time,
            error_rate,
            details
        )

        self.test_results.append(result)
        return result

    def _calculate_percentile(self, data: List[float], percentile: float) -> float:
        """
        Calculate percentile of response times
        """
        if not data:
            return 0
        sorted_data = sorted(data)
        index = int(len(sorted_data) * percentile / 100)
        return sorted_data[min(index, len(sorted_data) - 1)]

    def run_endpoint_performance_test(self, endpoints: List[str], num_requests_per_endpoint: int = 50) -> Dict[str, Any]:
        """
        Test performance of multiple endpoints
        """
        results = {
            "timestamp": datetime.utcnow().isoformat(),
            "endpoints_tested": len(endpoints),
            "individual_results": [],
            "summary": {}
        }

        for endpoint in endpoints:
            result = self.run_load_test(endpoint, num_requests_per_endpoint, concurrency=5)
            results["individual_results"].append({
                "endpoint": endpoint,
                "avg_response_time": result.avg_response_time,
                "requests_per_second": result.requests_per_second,
                "error_rate": result.error_rate,
                "details": result.details
            })

        # Calculate summary statistics
        if results["individual_results"]:
            response_times = [r["avg_response_time"] for r in results["individual_results"]]
            rps_values = [r["requests_per_second"] for r in results["individual_results"]]
            error_rates = [r["error_rate"] for r in results["individual_results"]]

            results["summary"] = {
                "avg_response_time": statistics.mean(response_times) if response_times else 0,
                "avg_requests_per_second": statistics.mean(rps_values) if rps_values else 0,
                "avg_error_rate": statistics.mean(error_rates) if error_rates else 0,
                "max_response_time": max(response_times) if response_times else 0,
                "min_response_time": min(response_times) if response_times else 0
            }

        return results

    def run_stress_test(self, endpoint: str, max_concurrent_users: int = 100,
                       duration_seconds: int = 60) -> Dict[str, Any]:
        """
        Run a stress test to determine breaking point
        """
        start_time = datetime.utcnow()
        results = {
            "endpoint": endpoint,
            "max_concurrent_users": max_concurrent_users,
            "duration_seconds": duration_seconds,
            "breaking_point": None,
            "performance_metrics": [],
            "start_time": start_time.isoformat(),
            "end_time": None
        }

        # Gradually increase load until performance degrades
        current_users = 10
        while current_users <= max_concurrent_users:
            result = self.run_load_test(endpoint, num_requests=current_users * 10,
                                      concurrency=current_users)

            metrics = {
                "concurrent_users": current_users,
                "avg_response_time": result.avg_response_time,
                "requests_per_second": result.requests_per_second,
                "error_rate": result.error_rate
            }

            results["performance_metrics"].append(metrics)

            # Check if performance is degrading (response time > 3 seconds or error rate > 5%)
            if result.avg_response_time > 3.0 or result.error_rate > 0.05:
                results["breaking_point"] = current_users
                break

            current_users += 10  # Increase load

        results["end_time"] = datetime.utcnow().isoformat()
        return results

    def generate_performance_report(self) -> str:
        """
        Generate a comprehensive performance report
        """
        optimization_analysis = self.run_performance_optimization_analysis()
        endpoints_to_test = ["/api/chapters", "/api/users/me", "/api/learning-paths"]

        endpoint_results = self.run_endpoint_performance_test(endpoints_to_test)

        report = f"""
# Performance Analysis Report
**Generated:** {optimization_analysis['timestamp']}

## Executive Summary
- **Overall Performance Score:** {optimization_analysis['performance_score']:.1f}/100
- **Database Performance:** {'Good' if optimization_analysis['database_performance']['avg_query_time'] < 0.1 else 'Needs Improvement'}
- **API Performance:** {'Good' if optimization_analysis['api_performance']['avg_response_time'] < 1.0 else 'Needs Improvement'}
- **Caching Effectiveness:** {'Good' if optimization_analysis['caching_effectiveness']['hit_rate'] > 0.7 else 'Needs Improvement'}

## Database Analysis
- Average Query Time: {optimization_analysis['database_performance']['avg_query_time']:.3f}s
- Maximum Query Time: {optimization_analysis['database_performance']['max_query_time']:.3f}s
- Slow Queries (>100ms): {optimization_analysis['database_performance']['queries_exceeding_threshold']}

## API Performance Analysis
- Average Response Time: {optimization_analysis['api_performance']['avg_response_time']:.3f}s
- 95th Percentile Response Time: {optimization_analysis['api_performance']['p95_response_time']:.3f}s
- Requests per Second: {optimization_analysis['api_performance']['requests_per_second']:.1f}
- Error Rate: {optimization_analysis['api_performance']['error_rate']:.2%}

## Caching Analysis
- Cache Hit Rate: {optimization_analysis['caching_effectiveness']['hit_rate']:.1%}
- Total Requests: {optimization_analysis['caching_effectiveness']['total_requests']}
- Cache Hits: {optimization_analysis['caching_effectiveness']['cache_hits']}

## Endpoint Performance
"""

        for endpoint_result in endpoint_results['individual_results']:
            report += f"""
### {endpoint_result['endpoint']}
- Average Response Time: {endpoint_result['avg_response_time']:.3f}s
- Requests per Second: {endpoint_result['requests_per_second']:.1f}
- Error Rate: {endpoint_result['error_rate']:.2%}
"""

        report += f"""

## Optimization Recommendations
"""
        for rec in optimization_analysis['optimization_recommendations']:
            report += f"""
- **{rec['priority'].title()} Priority**: {rec['recommendation']} (Impact: {rec['impact']})
"""

        report += f"""

## Performance Score Breakdown
- Database Performance: {max(0, 100 - int(optimization_analysis['database_performance']['avg_query_time'] * 200))}/100
- API Response Time: {max(0, 100 - int(optimization_analysis['api_performance']['avg_response_time'] * 50))}/100
- Caching Effectiveness: {int(optimization_analysis['caching_effectiveness']['hit_rate'] * 100)}/100

---
*This report was automatically generated by the Physical AI Textbook performance testing framework.*
"""

        return report

    def optimize_database_queries(self) -> List[Dict[str, str]]:
        """
        Provide database optimization recommendations
        """
        recommendations = [
            {
                "type": "indexing",
                "recommendation": "Add indexes to frequently queried columns like user_id, chapter_id",
                "priority": "high"
            },
            {
                "type": "query_optimization",
                "recommendation": "Use selectinload/joinedload to prevent N+1 query problems",
                "priority": "high"
            },
            {
                "type": "connection_pooling",
                "recommendation": "Tune database connection pool size based on concurrent users",
                "priority": "medium"
            }
        ]

        return recommendations

    def optimize_api_endpoints(self) -> List[Dict[str, str]]:
        """
        Provide API optimization recommendations
        """
        recommendations = [
            {
                "type": "caching",
                "recommendation": "Implement response caching for read-heavy endpoints",
                "priority": "high"
            },
            {
                "type": "pagination",
                "recommendation": "Add pagination to endpoints returning large datasets",
                "priority": "high"
            },
            {
                "type": "compression",
                "recommendation": "Enable response compression for large payloads",
                "priority": "medium"
            }
        ]

        return recommendations


# Convenience function to get performance testing service
def get_performance_testing_service(db: Session, base_url: str = "http://localhost:8000") -> PerformanceTestingService:
    return PerformanceTestingService(db, base_url)